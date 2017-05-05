#include "dmotion/GaitStateLib/GaitStateBase.hpp"

ros::NodeHandle* GaitStateBase::m_nh = nullptr;

void GaitStateBase::readOptions() {
//  auto& config = MotionConfigClient::getinstance()->config();
//  auto& robot = config.robot;
//  get_val(robot["stepnum"], stepNum);
//  get_val(robot["number"], m_robot_number);
//  get_val(robot["setupside_ankle_theta"],  m_setupside_ankle_theta);
//  get_val(robot["setupside_arm_theta"],  m_setupside_arm_theta);
//
//  get_val(robot["goalie_theta"], m_goalie_theta);
//  get_val(robot["goalie_bool"], m_goalie_bool);
//  get_val(robot["setupside_ankle_theta"], m_setupside_ankle_theta);
}

void GaitStateBase::loadGaitFile() {}

bool GaitStateBase::operator==(const GaitStateType type) {
  return this->type == type;
}

bool GaitStateBase::operator!=(const GaitStateType type) {
  return this->type != type;
}
