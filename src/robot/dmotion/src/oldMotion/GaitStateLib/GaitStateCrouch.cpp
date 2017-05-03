#include "GaitStateCrouch.hpp"

GaitStateCrouch::GaitStateCrouch(I_HumanRobot* robot)
    : GaitStateBase(1, CROUCH, robot) {
}

GaitStateCrouch::~GaitStateCrouch() {
}


void GaitStateCrouch::entry() {
  robot->staticEntry();
}

void GaitStateCrouch::execute() {
  LOG(DEBUG,
      "stand2crouch_stepnum is" << RobotPara::stand2crouch_stepnum << '\n');
  robot->doCrouchFromStand(RobotPara::stand2crouch_stepnum);
}

void GaitStateCrouch::exit() {
  robot->staticExit();
}
