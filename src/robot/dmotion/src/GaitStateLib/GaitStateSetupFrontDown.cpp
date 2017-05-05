#include "dmotion/GaitStateLib/GaitStateSetupFrontDown.hpp"
#include "dmotion/GaitStateManager.hpp"
#include <sstream>
using namespace std;
GaitStateSetupFrontDown::GaitStateSetupFrontDown(I_HumanRobot* robot,
                                                 GaitStateManager* manager)
    : GaitStateBase(SETUPFRONTDOWN, robot), manager(manager) {
  ROS_WARN("TODO load gait file");
}

GaitStateSetupFrontDown::~GaitStateSetupFrontDown() = default;

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
  } else {
    robot->dofirstStep();
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
    robot->runWalk(0, 0, 0);
  }
}