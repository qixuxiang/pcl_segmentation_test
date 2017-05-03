#include "GaitStateWenxi.hpp"

GaitStateWenxi::GaitStateWenxi(I_HumanRobot* robot)
    : GaitStateBase(3, WENXI, robot) {
  readOptions();
}

GaitStateWenxi::~GaitStateWenxi() {
}

void GaitStateWenxi::readOptions() {
  auto& config = MotionConfigClient::getinstance()->config();
  auto& robot = config.robot;
  get_val(robot["left_y_max"], length_max_l);
  get_val(robot["right_y_max"], length_max_r);
  get_val(robot["top_x_max"], length_max_f);
  get_val(robot["back_x_max"], length_max_b);
}

void GaitStateWenxi::execute() {
  // PPML_DEBUG("m_gait_vs size is %d , the 3's x number is %f ",
  // m_gait_vs.size() , m_gait_vs[2].gait_sx_);

  // PPML_DUBUG("wenxi normal");
  m_gait_sy = (m_gait_sy < length_max_l) ? m_gait_sy : length_max_l;
  m_gait_sy = (m_gait_sy > length_max_r) ? m_gait_sy : length_max_r;
  m_gait_sx = (m_gait_sx < length_max_f) ? m_gait_sx : length_max_f;
  m_gait_sx = (m_gait_sx > length_max_b) ? m_gait_sx : length_max_b;
  robot->runWalk(m_gait_sx, m_gait_sy, m_gait_st);
}
