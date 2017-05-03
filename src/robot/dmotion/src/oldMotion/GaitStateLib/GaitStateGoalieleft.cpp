#include "GaitStateGoalieleft.hpp"

GaitStateGoalieleft::GaitStateGoalieleft(I_HumanRobot* robot)
    : GaitStateBase(2, GOALIELEFT, robot) {
}

GaitStateGoalieleft::~GaitStateGoalieleft() {
}

void GaitStateGoalieleft::entry() {
  robot->m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void GaitStateGoalieleft::exit() {
  robot->m_robotCtrl.supportStatus = DOUBLE_BASED;
}

void GaitStateGoalieleft::execute() {
  // std::cout <<"goallieleft"<<std::endl;
  if (m_goalie_bool) {
    doLiftleftHand();
  } else {
    doGoalieLeft();
  }
  usleep(m_sleeptime);
  doRecover();
}

void GaitStateGoalieleft::doLiftleftHand() {
  RobotCtrl targetCtrl = robot->m_robotCtrl;

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
  targetCtrl.lh[1] = m_goalie_theta;

  int* dataArray = new int[MOTORNUM];
  double* handcsleft_15 = new double[m_stepnum];
  double* handcsright_17 = new double[m_stepnum];

  for (int i = 0; i < m_stepnum; i++) {
    robot->m_robotCtrl.num_left = m_stepnum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    dataArray[15 - 1] =
        handcsleft_15[i] * robot->m_motor_k[15 - 1] * robot->m_motor_zf[15 - 1];
    dataArray[17 - 1] = handcsright_17[i] * robot->m_motor_k[17 - 1] *
                        robot->m_motor_zf[17 - 1];
    robot->doTxTask(dataArray);
  }
}

void GaitStateGoalieleft::doGoalieLeft() {
  RobotCtrl targetCtrl = robot->m_robotCtrl;

  // ankle
  targetCtrl.la[2] = 3;  // bug fix
  targetCtrl.la[3] = RobotPara::la_r;
  targetCtrl.la[4] = RobotPara::la_p;
  targetCtrl.la[5] = 0;
  targetCtrl.ra[2] = 0;
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
  targetCtrl.lh[1] = m_goalie_theta;

  int* dataArray = new int[MOTORNUM];  // bug
  for (int i = 0; i < m_stepnum; i++) {
    robot->m_robotCtrl.num_left = m_stepnum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    robot->doTxTask(dataArray);
  }
}

void GaitStateGoalieleft::doRecover() {
  /* to do: crouch::getinstance()->robotctrl; */
  RobotCtrl targetCtrl = robot->m_robotCtrl;
  targetCtrl.la[2] = 3;
  targetCtrl.la[5] = m_setupside_ankle_theta;

  int* dataArray = new int[MOTORNUM];  // bug
  for (int i = 0; i < m_stepnum; i++) {
    robot->m_robotCtrl.num_left = m_stepnum - i;
    robot->getAngle_serial(targetCtrl, dataArray, 1);
    robot->doTxTask(dataArray);
  }
}
