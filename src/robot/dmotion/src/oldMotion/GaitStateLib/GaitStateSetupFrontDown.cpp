#include "GaitStateSetupFrontDown.hpp"
#include <sstream>

GaitStateSetupFrontDown::GaitStateSetupFrontDown(I_HumanRobot* robot,
                                                 GaitStateManager* manager)
    : GaitStateBase(2, SETUPFRONTDOWN, robot), manager(manager) {
  loadGaitFile();
}

GaitStateSetupFrontDown::~GaitStateSetupFrontDown() {
}

void GaitStateSetupFrontDown::entry() {
  robot->doCrouchFromStand(20);
  robot->m_robotCtrl.supportStatus = DOUBLE_BASED;
  robot->m_robotCtrl.setAutoMode();
  int step = 150;
  int* dataArray = new int[MOTORNUM];

  robot->m_robotCtrl.cm[0] = RobotPara::hipheight - RobotPara::cm_dx;
  robot->m_robotCtrl.cm[2] = -RobotPara::cm_dx;
  robot->m_robotCtrl.cm[4] = 90 + RobotPara::cm_p;
  robot->m_robotCtrl.la[4] = 90;
  robot->m_robotCtrl.ra[4] = 90;

  RobotCtrl target_robotCtrl = robot->m_robotCtrl;
  /// step1
  target_robotCtrl.la[1] =
      RobotPara::hip_distance / 2 + RobotPara::yzmp;  // bug
  target_robotCtrl.ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp;

  LOG(DEBUG, length);
  for (int i = 0; i < length; i++) {
    LOG(DEBUG, i);
    step = data[0][i];
    target_robotCtrl.cm[0] = data[1][i];  // 17;
    target_robotCtrl.cm[2] = data[2][i];  // 6;
    target_robotCtrl.cm[4] = data[3][i];  // 100;
    target_robotCtrl.la[4] = data[4][i];  // 30;
    target_robotCtrl.ra[4] = data[5][i];  // 30;
    target_robotCtrl.lh[0] = data[6][i];  // 6;
    target_robotCtrl.lh[1] = data[7][i];  //-4;
    target_robotCtrl.rh[0] = data[8][i];  // 6;
    target_robotCtrl.rh[1] = data[9][i];  //-4;

    for (int i = 0; i < step; i++) {
      robot->m_robotCtrl.num_left = step - i;
      robot->getAngle_serial(target_robotCtrl, dataArray, 1);
      robot->doTxTask(dataArray);
    }
  }
  robot->m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void GaitStateSetupFrontDown::execute() {
  robot->doCrouchFromStand(30);
}

void GaitStateSetupFrontDown::exit() {
  if (*manager->goal_gaitState == STANDUP ||
      *manager->goal_gaitState == CROUCH ||
      *manager->goal_gaitState == GOALIELEFT ||
      *manager->goal_gaitState == GOALIERIGHT ||
      *manager->goal_gaitState == GOALIEMID ||
      *manager->goal_gaitState == SETUPBACKDOWN ||
      *manager->goal_gaitState == SETUPFRONTDOWN ||
      *manager->goal_gaitState == SETUPLEFTDOWN ||
      *manager->goal_gaitState == SETUPRIGHTDOWN) {
    LOG(DEBUG, "exit() do nothing. \n");
  } else {
    LOG(DEBUG, "do first step");
    robot->dofirstStep();
    LOG(DEBUG, "do first step done");
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
    LOG(DEBUG, "exit() do firststep.\n");
  }
}

void GaitStateSetupFrontDown::loadGaitFile() {
  string fileAddress = "./config/gaitData";
  stringstream tempRobotNumber;
  string tempString;
  tempRobotNumber << m_robot_number;
  tempRobotNumber >> tempString;

  auto jiangyin = fileAddress + tempString + "/FrontDown.txt";

  LOG(DEBUG, jiangyin);
  if (!loadFile(jiangyin.c_str(), data)) {
    LOG(FATAL, "fail to load gait file: " << jiangyin);
  }
}

bool GaitStateSetupFrontDown::loadFile(const char* filename, double** t_data) {
  file.clear();
  file.open(filename, ios_base::in);

  if (file.fail()) {
    return false;
  }
  if (!loadData(t_data))
    return false;
  if (file.is_open()) {
    file.close();
  }
  return true;
}

bool GaitStateSetupFrontDown::loadData(double** t_data) {
  double temp;

  length = 0;
  while (file >> temp) {
    length++;
  }
  file.clear();
  length = length / 10;  // bug
  for (int i = 0; i < 10; i++) {
    if (data[i]) {
      data[i] = NULL;
    }
    data[i] = new double[length];
  }

  file.seekg(ios_base::beg);

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < length; j++) {
      file >> temp;
      data[i][j] = temp;
    }
  }
  return true;
}
