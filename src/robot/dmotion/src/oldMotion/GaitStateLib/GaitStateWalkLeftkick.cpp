#include "GaitStateWalkLeftkick.hpp"

GaitStateWalkLeftkick::GaitStateWalkLeftkick(I_HumanRobot* robot)
    : GaitStateBase(3, WALKLEFTKICK, robot) {
  readOptions();
}

GaitStateWalkLeftkick::~GaitStateWalkLeftkick() {
}

void GaitStateWalkLeftkick::readOptions() {
  auto& config = MotionConfigClient::getinstance()->config();
  auto& robot = config.robot;
  get_val(robot["kicklength_l"], m_gait_sx);
  get_val(robot["kickheight_l"], m_AnkleH_mid_l);
  get_val(robot["kickexcute_l"], m_kickexcute_l);
}

void GaitStateWalkLeftkick::execute() {
  robot->m_AnkleH_mid_l = RobotPara::ah_ml_mid;
  robot->runWalk(m_kickexcute_l, 0, m_gait_angle / 3);
  robot->runWalk(m_kickexcute_l, 0, m_gait_angle / 3);
}

void GaitStateWalkLeftkick::entry() {
  if (robot->m_robotCtrl.supportStatus == RIGHT_BASED) {
    robot->runWalk(0, 0, m_gait_angle / 3);
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);  // feature
    robot->m_leftkick_flag = 1;
    robot->m_AnkleH_mid_l = m_AnkleH_mid_l;
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);
    robot->m_leftkick_flag = 0;
  } else {
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);
    robot->m_leftkick_flag = 1;
    robot->m_AnkleH_mid_l = m_AnkleH_mid_l;
    robot->runWalk(m_gait_sx, 0, m_gait_angle / 3);  // feature
    robot->m_leftkick_flag = 0;
  }
}
