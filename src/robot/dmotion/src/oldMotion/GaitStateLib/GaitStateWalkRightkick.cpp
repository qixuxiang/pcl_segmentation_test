#include "GaitStateWalkRightkick.hpp"

GaitStateWalkRightkick::GaitStateWalkRightkick(I_HumanRobot* robot)
    : GaitStateBase(3, WALKLEFTKICK, robot) {
  readOptions();
}

GaitStateWalkRightkick::~GaitStateWalkRightkick() {}

void GaitStateWalkRightkick::readOptions() {
  auto& config = MotionConfigClient::getinstance()->config();
  auto& robot = config.robot;
  get_val(robot["kicklength_r"],   m_gait_sx );
  get_val(robot["kickheight_r"],   m_AnkleH_mid_r );
  get_val(robot["kickexcute_r"],   m_kickexcute_r );
}

void GaitStateWalkRightkick::execute() {
  robot->m_AnkleH_mid_r = RobotPara::ah_mr_mid;
  robot->runWalk(m_kickexcute_r, 0, m_gait_angle / 3);
  robot->runWalk(m_kickexcute_r, 0, m_gait_angle / 3);
}

void GaitStateWalkRightkick::entry() {
  if (robot->m_robotCtrl.supportStatus == RIGHT_BASED) {
    robot->runWalk(0, 0, m_gait_angle / 3);
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);  // feature
    robot->m_leftkick_flag = 1;
    robot->m_AnkleH_mid_r = m_AnkleH_mid_r;
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);
    robot->m_leftkick_flag = 0;
  } else {
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);
    robot->m_leftkick_flag = 1;
    robot->m_AnkleH_mid_r = m_AnkleH_mid_r;
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);  // feature
    robot->m_leftkick_flag = 0;
  }
}
