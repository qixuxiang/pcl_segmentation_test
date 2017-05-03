#include "GaitStateSetupRightDown.hpp"
GaitStateSetupRightDown::GaitStateSetupRightDown(I_HumanRobot* robot,
                                                 GaitStateManager* manager)
    : GaitStateBase(2, SETUPRIGHTDOWN, robot), manager(manager) {
  length = robot->loadGaitFile("RightDown.txt", data, 10);
}

GaitStateSetupRightDown::~GaitStateSetupRightDown() {
}

/* use less to be cleaned */
void GaitStateSetupRightDown::readOptions() {
  auto& config = MotionConfigClient::getinstance()->config();
  auto& robot = config.robot;

  get_val(robot["setupside_ankle_theta"],   m_setupside_ankle_theta);
  get_val(robot["setupside_arm_theta"],   m_setupside_arm_theta);
}

void GaitStateSetupRightDown::entry() {
  // must get the information from the gyroscope
  robot->doCrouchFromStand(20);
  robot->m_robotCtrl.setAutoMode();

  int step;
  step = 150;
  int* dataArray = new int[MOTORNUM];  //[MOTORNUM];
  // cout<<m_robot->m_hipHeight<<" "<<m_robot->m_cm_dx<<endl;

  RobotCtrl target_robotCtrl;
  target_robotCtrl = robot->m_robotCtrl;
  /// step1
  target_robotCtrl.la[1] =
      RobotPara::hip_distance / 2 + RobotPara::yzmp;  // bug
  target_robotCtrl.ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp;

  for (int i = 0; i < length; i++) {
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
}

void GaitStateSetupRightDown::execute() {
  robot->doCrouchFromStand(30);
}

void GaitStateSetupRightDown::exit() {
  if (*manager->goal_gaitState == STANDUP ||
      *manager->goal_gaitState == CROUCH ||
      *manager->goal_gaitState == GOALIELEFT ||
      *manager->goal_gaitState == GOALIERIGHT ||
      *manager->goal_gaitState == GOALIEMID ||
      *manager->goal_gaitState == SETUPBACKDOWN ||
      *manager->goal_gaitState == SETUPFRONTDOWN ||
      *manager->goal_gaitState == SETUPLEFTDOWN ||
      *manager->goal_gaitState == SETUPRIGHTDOWN) {

    LOG(DEBUG, "[Crouch] exit() do nothing. \n");
    // cout << "do nothing" << endl;
  } else {
    // cout << "do first step >>>>" << endl;
    robot->dofirstStep();
    // cout << "first step done" << endl;
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
    LOG(DEBUG, "[Crouch] exit() do firststep.\n");
  }
}
