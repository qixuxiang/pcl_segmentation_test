#include "dmotion/GaitStateLib/GaitStateCrouch.hpp"

GaitStateCrouch::GaitStateCrouch(I_HumanRobot* robot)
    : GaitStateBase(CROUCH, robot) {
}

GaitStateCrouch::~GaitStateCrouch() = default;

void GaitStateCrouch::entry() {
  robot->staticEntry();
}

void GaitStateCrouch::execute() {
  ROS_DEBUG("Crouch execute");
  robot->doCrouchFromStand(RobotPara::stand2crouch_stepnum);
}

void GaitStateCrouch::exit() {
  robot->staticExit();
}
