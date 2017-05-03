#include "GaitStateGoalieright.hpp"

GaitStateGoalieright::GaitStateGoalieright(I_HumanRobot* robot)
    : GaitStateBase(2, GOALIERIGHT, robot) {
}

GaitStateGoalieright::~GaitStateGoalieright() {
}



void GaitStateGoalieright::execute() {
  if (m_goalie_bool) {
    doLiftrightHand();
  } else {
    doGoalieRight();
  }

  doRecover();
}

void GaitStateGoalieright::doLiftrightHand() {
  RobotCtrl targetCtrl = robot->m_robotCtrl;

  targetCtrl = robot->m_robotCtrl;
  // ankle
  targetCtrl.la[2] = 0;
  targetCtrl.la[3] = RobotPara::la_r;
  targetCtrl.la[4] = RobotPara::la_p;
  targetCtrl.la[5] = 0;
  targetCtrl.ra[2] = 0;
  targetCtrl.ra[3] = RobotPara::ra_r;
  targetCtrl.ra[4] = RobotPara::ra_p;
  targetCtrl.ra[5] = 0;

  // cm
  targetCtrl.cm[0] = 0;
  targetCtrl.cm[2] = RobotPara::hipheight;
  targetCtrl.cm[3] = RobotPara::cm_r;
  targetCtrl.cm[4] = RobotPara::cm_p;
  targetCtrl.cm[5] = RobotPara::cm_y;
  targetCtrl.cm_dxy[0] = RobotPara::cm_dx;
  targetCtrl.cm_dxy[1] = RobotPara::cm_dy;

  // arm
  targetCtrl.lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
  targetCtrl.rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
  targetCtrl.rh[1] = m_goalie_theta;

  int* dataArray = new int[MOTORNUM];  // bug
  for (int i = 0; i < stepNum; i++) {
    robot->m_robotCtrl.num_left = stepNum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    robot->doTxTask(dataArray);
  }
}

void GaitStateGoalieright::doGoalieRight() {
  RobotCtrl targetCtrl = robot->m_robotCtrl;

  // ankle
  targetCtrl.la[2] = 0;  // bug fix
  targetCtrl.la[3] = RobotPara::la_r;
  targetCtrl.la[4] = RobotPara::la_p;
  targetCtrl.la[5] = 0;
  targetCtrl.ra[2] = 3;
  targetCtrl.ra[3] = -0;
  targetCtrl.ra[4] = RobotPara::ra_p;
  targetCtrl.ra[5] = 0;
  // cm
  targetCtrl.cm[0] = 0;
  targetCtrl.cm[2] = RobotPara::hipheight;
  targetCtrl.cm[3] = RobotPara::cm_r;
  targetCtrl.cm[4] = RobotPara::cm_p;
  targetCtrl.cm[5] = 0;
  targetCtrl.cm_dxy[0] = RobotPara::cm_dx;
  targetCtrl.cm_dxy[1] = RobotPara::cm_dy;
  // arm
  targetCtrl.lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
  targetCtrl.rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
  targetCtrl.rh[1] = m_goalie_theta;

  int* dataArray = new int[MOTORNUM];  // bug

  for (int i = 0; i < stepNum; i++) {
    robot->m_robotCtrl.num_left = stepNum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    robot->doTxTask(dataArray);
  }
}

void GaitStateGoalieright::doRecover() {
  RobotCtrl targetCtrl = robot->m_robotCtrl;
  targetCtrl.la[2] = 3;
  targetCtrl.la[5] = m_setupside_ankle_theta;

  int* dataArray = new int[MOTORNUM];  // bug
  for (int i = 0; i < stepNum; i++) {
    robot->m_robotCtrl.num_left = stepNum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    robot->doTxTask(dataArray);
  }
}
