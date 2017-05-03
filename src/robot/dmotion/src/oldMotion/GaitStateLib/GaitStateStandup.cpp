#include "GaitStateStandup.hpp"
#include "GaitStateCrouch.hpp"

// extern attemptingShutdown;
GaitStateStandup::GaitStateStandup(I_HumanRobot* robot)
    : GaitStateBase(1, STANDUP, robot) {
}

GaitStateStandup::~GaitStateStandup() {
}



//!?
void GaitStateStandup::entry() {

  // robot->staticEntry();

  // RobotCtrl ori;
  // double ankle_distance_small = ori.la[1] - ori.ra[1];
  // double current_dis = robot->m_robotCtrl.la[1] - robot->m_robotCtrl.ra[1];

  // LOG(INFO, "stand ankle is %f %f", ankle_distance_small, current_dis);
  // if (attemptingShutdown && robot->m_robotCtrl.supportStatus == DOUBLE_BASED &&
  //     current_dis > ankle_distance_small + 0.1) {
  //   LOG(INFO, "1 attemptingShutdown stand do dofirstStep");
  //   robot->dofirstStep();
  //   RobotPara::step_x_amend = 0;
  //   robot->runWalk(0, 0, 0);
  //   robot->runWalk(0, 0, 0);
  //   RobotPara::ankle_distance = RobotPara::hip_distance;
  //   robot->runWalk(0, 0, 0);
  //   RobotPara::ra_p = 0;
  //   RobotPara::la_p = 0;
  //   robot->m_robotCtrl.supportStatus = DOUBLE_BASED;

  // } else if (attemptingShutdown &&
  //            robot->m_robotCtrl.supportStatus !=
  //                DOUBLE_BASED)  //&&robot->m_robotCtrl.supportStatus ==
  //                               //DOUBLE_BASED)
  // {
  //   // robot->runWalk(0, 0, 0);
  //   LOG(INFO, "2");
  //   robot->runWalk(0, 0, 0);
  //   robot->runWalk(0, 0, 0);
  //   RobotPara::step_x_amend = 0;
  //   robot->runWalk(0, 0, 0);
  //   RobotPara::ankle_distance = RobotPara::hip_distance;
  //   robot->runWalk(0, 0, 0);
  //   RobotPara::ra_p = 0;
  //   RobotPara::la_p = 0;
  //   robot->m_robotCtrl.supportStatus = DOUBLE_BASED;

  // } else if (attemptingShutdown) {
  //   RobotPara::ra_p = 0;
  //   RobotPara::la_p = 0;
  //   robot->staticEntry();
  // } else {
  //   LOG(INFO, "3");
  //   robot->staticEntry();
  // }
  robot->staticEntry();
}

void GaitStateStandup::execute() {
  // std::cout <<"stand up execute" <<std::endl;
  robot->doStandFromCrouch(stepNum * 4);
  // robot->doCrouchFromStandMotor(stepNum*5);
}

void GaitStateStandup::exit() {
  robot->doCrouchFromStand(RobotPara::stepnum * 2);  // crouch first
  robot->staticExit();
  // std::cout <<"stand up exit" <<std::endl;
}
