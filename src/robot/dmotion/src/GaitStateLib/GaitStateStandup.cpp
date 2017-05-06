#include "dmotion/GaitStateLib/GaitStateStandup.hpp"

GaitStateStandup::GaitStateStandup(I_HumanRobot* robot)
    : GaitStateBase(STANDUP, robot) {

}

GaitStateStandup::~GaitStateStandup() = default;

void GaitStateStandup::entry() {
  robot->staticEntry();
}

void GaitStateStandup::execute() {
  robot->doStandFromCrouch(stepNum * 4);
}

void GaitStateStandup::exit() {
  robot->doCrouchFromStand(RobotPara::stepnum * 2);  // crouch first
  robot->staticExit();
}
