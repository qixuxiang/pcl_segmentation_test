#include "dmotion/dmotion.hpp"
#include <ros/ros.h>

namespace dmotion {
static const int MOTION_FREQ = 100;
DMotion::DMotion(ros::NodeHandle* nh)
  : DProcess(MOTION_FREQ, true)
  , m_nh(nh)
  , m_manager(nh)
{
    m_sub = m_nh->subscribe("/humanoid/ActionCommand", 1, &DMotion::callback, this);
    // TODO(MWX): feedback topic
}

DMotion::~DMotion() = default;

void
DMotion::tick()
{
    m_manager.checkNewCommand(m_cmd);
    m_manager.tick();
}

void
DMotion::callback(const ActionCmd::ConstPtr& msg)
{
    m_cmd = *msg;
}

void
DMotion::prepareShutdown() {
    m_sub.shutdown();
    m_cmd.gait_type = ActionCmd::STANDUP;
    m_cmd.cmd_head.y = m_cmd.cmd_head.z = 0;
    for(int i = 0; i < 2; ++i) {
        printf("DMotion shutting down >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        tick();
    }
}
}
