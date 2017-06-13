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
    for(int i = 0; i < 3; ++i) {
        printf("shuting down >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        tick();
    }
}
}
