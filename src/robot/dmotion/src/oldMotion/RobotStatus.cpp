#include "RobotStatus.hpp"
#include "misc/utils/logger/logger.hpp"
#include "motion/MotionConfig.hpp"
#include <fstream>


//std::ofstream compass_log("./compass_filter.log"););

using namespace std;
using namespace dancer2050;

using namespace ActionCommand;

namespace compass_config {
  double x0;
  double y0;
  double angle0;
  double phi;
  double ratio;
}

double prev_field_angle = 0;
int current_compass_range = 0; // (-180 ~ 180) + x * 180

RobotStatus::RobotStatus() {
  LOG(INFO, "        [ Constructing RobotStatus" );
  //
  readOptions();
  initMotor();
  m_last_angle_z = 0;
  m_isRun = false;
  m_comInfo = new comEstimator();
  compass_filter = One_D_Filter(1000);
  all = All(Head(), Body());
  LOG(INFO, "        RobotStatus Constructed ]" );
}

RobotStatus::~RobotStatus() { }

angledataDebug RobotStatus::getAngledata() {
  angledataDebug temp = m_angledata;
  return temp;
}

GyroData RobotStatus::getGdata() { return m_gypdata; }

void RobotStatus::setGdata(GyroData gdata) {
  m_gypdata = gdata;
  // update body angle
  m_angledata.angleX += gdata.GYPO[0] * MOTION::sampletime;
  m_angledata.angleY += gdata.GYPO[1] * MOTION::sampletime;
  m_angledata.angleZ += gdata.GYPO[2] * MOTION::sampletime;
  m_angledata.corresCycle = gdata.corresCycle;
  // update offset
  m_offset.offsetX +=
    gdata.ACCL[0] * MOTION::sampletime * MOTION::sampletime * 100;
  m_offset.offsetY +=
    gdata.ACCL[1] * MOTION::sampletime * MOTION::sampletime * 100;
  m_offset.corresCycle = gdata.corresCycle;
}

void RobotStatus::updateEularAngle() {
  VecPos eular;
  GyroData cur_gyro_data = m_gypdata;
  m_comInfo->m_filter->estimate(cur_gyro_data.GYPO[0], cur_gyro_data.GYPO[1],
                                cur_gyro_data.GYPO[2], cur_gyro_data.ACCL[0],
                                cur_gyro_data.ACCL[1], cur_gyro_data.ACCL[2],
                                MOTION::sampletime);
  eular.m_x = m_comInfo->m_filter->eular[0];
  eular.m_y = m_comInfo->m_filter->eular[1];
  m_eular_deque.push_front(eular);
  if (m_eular_deque.size() > 50)  // 1s
  {
    m_eular_deque.pop_back();
  }
  return;
}

stabilityStatus RobotStatus::checkStableState() {
  /// get eular angle
  VecPos eular = m_eular_deque.front();
  double temp;

  temp = eular.m_x;
  eular.m_x = eular.m_y;
  eular.m_y = temp * -1;

  // if( c_gyro_type =="MPU6000")
  // {
  //     if(c_robot_version == "2012")
  //     {
  //         temp = eular.m_x;
  //         eular.m_x = eular.m_y;
  //         eular.m_y = temp*-1;
  //     }
  //     else if(c_robot_version == "2012.5")
  //     {
  //         eular.m_x*=-1;
  //         eular.m_y*=-1;
  //     }
  //     else// "2013"
  //     {
  //         eular.m_x*=-1;
  //         eular.m_y*=-1;
  //     }
  // }

  eular.m_x *= -1;
  eular.m_y *= -1;

  eular.m_x *= (180 / M_PI);
  eular.m_y *= (180 / M_PI);
  static int frontcount, backcount, leftcount, rightcount, stablecount;
  if (abs(eular.m_x) > 40 || abs(eular.m_y) > 40) {
    // m_desPlat.m_x = 0;
    // m_desPlat.m_y = 0;
    // m_PlatRotateStep = 12;

    if (eular.m_x > 60) {
      frontcount++;
    } else {
      frontcount = 0;
    }
    if (eular.m_x < -60) {
      backcount++;
    } else {
      backcount = 0;
    }
    if (eular.m_y > 60) {
      rightcount++;
    } else {
      rightcount = 0;
    }
    if (eular.m_y < -60) {
      leftcount++;
    } else {
      leftcount = 0;
    }
  } else {
    stablecount++;
    frontcount = backcount = leftcount = rightcount = 0;
  }
  if (frontcount > 10) {
    m_bStable = frontdown;
  } else if (backcount > 10) {
    m_bStable = backdown;
  } else if (leftcount > 10) {
    m_bStable = leftdown;
  } else if (rightcount > 10) {
    m_bStable = rightdown;
  } else if (stablecount > 10) {
    m_bStable = stable;
  }
  return m_bStable;
}

void RobotStatus::updateDeltaDist(const VecPos &D, double deltaBodyAngle) {
  int k = -1;
  deltaBodyAngle = k * deltaBodyAngle;
  double rx = D.m_x / (deltaBodyAngle / 180 * M_PI);
  double ry = D.m_y / (deltaBodyAngle / 180 * M_PI);
  if (deltaBodyAngle == 0) {
    m_deltaDist.m_x += D.m_x;
    m_deltaDist.m_y += D.m_y;
  } else if (m_deltaDist.m_angle == 0 && m_deltaDist.m_x == 0 &&
             m_deltaDist.m_y == 0) {
    m_deltaDist.m_x = rx * sin(deltaBodyAngle / 180 * M_PI) +
                      ry * (1 - cos(deltaBodyAngle / 180 * M_PI));
    m_deltaDist.m_y = rx * (1 - cos(deltaBodyAngle / 180 * M_PI)) +
                      ry * sin(deltaBodyAngle / 180 * M_PI);
    m_deltaDist.m_angle = deltaBodyAngle;
  } else {
    VecPos addValue =
      VecPos(rx * sin(deltaBodyAngle / 180 * M_PI) +
             ry * (1 - cos(deltaBodyAngle / 180 * M_PI)),
             rx * (1 - cos(deltaBodyAngle / 180 * M_PI)) +
             ry * sin(deltaBodyAngle / 180 * M_PI));
    addValue.rotate(m_deltaDist.m_angle);
    m_deltaDist.m_x += addValue.m_x;
    m_deltaDist.m_y += addValue.m_y;
    m_deltaDist.m_angle += deltaBodyAngle;
  }
}

initdataDebug RobotStatus::getMotorinit() { return m_motorini; }

initdataDebug RobotStatus::getRawMotorInit() { return raw_motorini; }

void RobotStatus::readOptions() {
  LOG(DEBUG, "            ( RobotStatus reading options" );

  auto& config = MotionConfigClient::getinstance()->config();

  robotnumber = as<int>(config.robot["number"]);

  std::stringstream num;
  for (int i = 0; i < MOTORNUM; i++) {
    num.str("");
    num << i + 1;

    int tmp = as<int>(config.motor[num.str() + ".k"]);

    // get_val(, tmp);

    if (tmp == 4096) {
      k_tmp[i] = tmp / 360.0;
    } else if (tmp == 1024) {
      k_tmp[i] = tmp / 360.0;
    } else {
      LOG(ERROR, "RobotStatus read motor config failed" );
      throw std::runtime_error("mx? rx? or other fu** dynamixel motor? ");
    }
  }
  LOG(DEBUG, "            RobotStatus read options succeed )" );
}


/*
 * update motor init
 */
void RobotStatus::updateMotor(int id, int value) {
  id -= 1;
  if (id > 20 || id < 0) {
    cout << __func__ << " no such motor id: " << id << endl;
  };


  if (value < 0 || value > 360 - 1) {
    cout << __func__ << " update Motor out of range id: " << id << " delta: " << value << endl;
  } else {
//    m_motorini.initial[id] = value;
    raw_motorini.initial[id] = value;
    m_motorini.initial[id] = (int) (k_tmp[id] * raw_motorini.initial[id]);
  }
}

/**
 * save current motor init data to config
 */
void RobotStatus::saveMotor() {
  std::stringstream robotnum;
  robotnum << robotnumber;
  std::string x = std::string("./config/gaitData") + robotnum.str() +
                  std::string("/initialDataConfig.txt");
//  string x = "initDataTest.txt";
  ofstream initial_file(x.c_str());

  initial_file << "//robot" << robotnum.str() << endl
  << "////1----2---3---4---5---6---7---8---9--10--11--12--13--14--15--16--17--18--21--22" << endl;
  initial_file << "\\\\";
  for (int i = 0; i < MOTORNUM; i++) {
    initial_file << raw_motorini.initial[i] << " ";
  }
  initial_file << endl;
  initial_file.close();
}

bool RobotStatus::initMotor() {
  LOG(DEBUG, "----------------- init Motor -----------------");
  std::stringstream robotnum;
  robotnum << robotnumber;
  std::string init_file = std::string("./config/gaitData") + robotnum.str() +
                  std::string("/initialDataConfig.txt");
  ifstream initial_file(init_file.c_str());

  char dataGot[200];
  double initial_tmp[MOTORNUM];

  if (initial_file.fail()) {
    LOG(ERROR, "can't open " << init_file );
    return false;
  }

  /* 100 lines most */
  int k = 100;
  while (k--) {
    initial_file.getline(dataGot, 200);
    if (dataGot[0] == '\\' && dataGot[1] == '\\') {
      stringstream conv;
      conv << &dataGot[2];
      for (int i = 0; i < MOTORNUM; i++) {
        conv >> initial_tmp[i];  // motorini.initial[i];
//        conv >> raw_motorini.initial[i];
        raw_motorini.initial[i] = initial_tmp[i];
        m_motorini.initial[i] = (int) (k_tmp[i] * raw_motorini.initial[i]);
//         cout << i << ": " << m_motorini.initial[i] << " " << raw_motorini.initial[i] << " " << k_tmp[i] << endl;
        if (m_motorini.initial[i] < 0 ||
            m_motorini.initial[i] > k_tmp[i] * 360 - 1) {
          throw std::runtime_error(
            "initialFile value error.notice the file is "
              "gaitData(robotnumber)/initialDataConfig.txt. the value is in "
              "unit angle about 180");
        }
      }
      break;
    } else if (dataGot[0] == '/' && dataGot[1] == '/' && dataGot[2] == 'r' &&
               dataGot[3] == 'o' && dataGot[4] == 'b' && dataGot[5] == 'o' &&
               dataGot[6] == 't') {
    }
    if (k == 0) {
      LOG(ERROR, "transitHub::initialData load error!" );
      cout << "transitHub::initialData load error!" << endl;
      return false;
    }
  }
  return true;
}

VecPos RobotStatus::getEularAngle() {
  VecPos retv;
  if (m_eular_deque.size() > 0) {
    retv = m_eular_deque.front();
  } else {
    retv = VecPos(0, 0);
  }

  retv.m_x *= -1;
  retv.m_y *= -1;

  return retv;
}

void RobotStatus::setCompassData(CompassData temp) {
  m_compassdata = temp;
}

CompassData RobotStatus::getCompassData() {
  return m_compassdata;
}

double RobotStatus::getCurrentFieldAngle() {
  auto angle = getFieldAngle(m_compassdata.x, m_compassdata.y, compass_config::x0, compass_config::y0, compass_config::angle0,
                       compass_config::phi, compass_config::ratio);

  auto filtered = compass_filter.update(m_deltaDist.m_angle, angle);

//  compass_log << angle << " " << filtered << " " << m_deltaDist.m_angle << endl;

  return AngleNormalization(filtered);
//  return angle;
}

deltadataDebug RobotStatus::checkDeltaDist() {
  deltadataDebug deltaDist = m_deltaDist;

  m_deltaDist.m_angle = 0;
  m_deltaDist.m_x = 0;
  m_deltaDist.m_y = 0;

//  if (deltaDist.m_x > 0) {
//    deltaDist.m_x *= forward_k;
//  } else {
//    deltaDist.m_x *= back_k;
//  }
//
//  if (deltaDist.m_y > 0) {
//    deltaDist.m_y *= left_k;
//  } else {
//    deltaDist.m_y *= right_k;
//  }

  return deltaDist;
}


double getFieldAngle(double x, double y, double x0, double y0, double angle0, double phi, double ratio) {

//  double xhead = 0;
//  double yhead = 0;

//  xhead = head_r * sin(-curyaw * M_PI / 180);
//  yhead = head_r * cos(curyaw * M_PI / 180);

  double cs = cos(-phi);  // cos and sin of -phi0
  double si = sin(-phi);  // cos and sin of -phi0
  double x00, x11;
  double y00, y11;

  // minus original shift
  x00 = x - x0;
  y00 = y - y0;

  // rotate (-phi)
  x11 = cs * x00 - si * y00;
  y11 = si * x00 + cs * y00;

  // remove ratio from eclipse to circle
  x11 /= ratio;

  double angle;
  angle = std::atan2(y11, x11);
  angle = (angle + phi) * 180 / M_PI - angle0;
//  cout << "x: " << x << " y: " << y << " x0: " << x0 << " y0: " << y0 << " angle0: " << angle0 << " phi: " << phi << " ratio: " << ratio << " result: " << AngleNormalization(angle)<< endl;
//  return angle;
  angle =  AngleNormalization(angle);
  auto diff = angle - prev_field_angle;


  if (diff < -250){
    current_compass_range += 1;
  } else if(diff > 250) {
    current_compass_range -= 1;
  }

  prev_field_angle = angle;

  angle += current_compass_range * 360;
  return angle;
}

void update_compass_config(double x0, double y0, double angle0, double phi0, double ratio) {
  LOG(INFO, "compass config updated" );
  LOG(INFO, "x0: " << x0 << " y0: " << y0 << " angle0: " << angle0 << " phi0: " << phi0 << " ratio: "<< ratio );

  compass_config::x0 = x0;
  compass_config::y0 = y0;
  compass_config::angle0 = angle0;
  compass_config::phi = phi0;
  compass_config::ratio = ratio;
}
