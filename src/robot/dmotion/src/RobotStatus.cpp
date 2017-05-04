#include "dmotion/RobotStatus.hpp"
#include <ros/ros.h>
#include <fstream>

using namespace std;
RobotStatus::RobotStatus() {
  readOptions();
  initMotor();
  m_last_angle_z = 0;
  m_isRun = false;
  m_comInfo = new comEstimator();
  compass_filter = One_D_Filter(1000);
}

RobotStatus::~RobotStatus() {}

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
//  std::stringstream num;
//  for (int i = 0; i < MOTORNUM; i++) {
//    num.str("");
//    num << i + 1;
//
//    int tmp = as<int>(config.motor[num.str() + ".k"]);
//
//    // get_val(, tmp);
//
//    if (tmp == 4096) {
//      k_tmp[i] = tmp / 360.0;
//    } else if (tmp == 1024) {
//      k_tmp[i] = tmp / 360.0;
//    } else {
//      LOG(ERROR, "RobotStatus read motor config failed" );
//      throw std::runtime_error("mx? rx? or other fu** dynamixel motor? ");
//    }
//  }
}

void RobotStatus::updateMotor(int id, int value) {
  id -= 1;
  if (id > 20 || id < 0) {
    ROS_ERROR("No such motor id: %d", id);
  }

  if (value < 0 || value > 360 - 1) {
    ROS_WARN("Update Motor out of range id: %d delta: %d", id, value);
  } else {
    raw_motorini.initial[id] = value;
    m_motorini.initial[id] = (int) (k_tmp[id] * raw_motorini.initial[id]);
  }
}

bool RobotStatus::initMotor() {
  ROS_INFO("------------------------------- init motor -------------------------------");
  std::stringstream robotnum;
  robotnum << robotnumber;
  std::string init_file = std::string("./config/gaitData") + robotnum.str() + std::string("/initialDataConfig.txt");
  ifstream initial_file(init_file.c_str());

  char dataGot[200];
  double initial_tmp[MOTORNUM];

  if (initial_file.fail()) {
    ROS_ERROR("Can't open %s", init_file.c_str());
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
      ROS_ERROR("transitHub::initialData load error!");
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
