#include "dmotion/BehaviorRecv.hpp"

namespace dmotion {
BehaviorRecv::BehaviorRecv(ros::NodeHandle* n, DMotion *d)
    : DProcess(50, false), m_nh(n), m_dmotion(d) {

    m_sub = m_nh->subscribe("/humanoid/ActionCommand", 1, &BehaviorRecv::callback, this);
}

void BehaviorRecv::callback(const ActionCommand::ConstPtr& msg) {
    m_dmotion->setCmd(*msg);
}

void BehaviorRecv::tick() {
    // pass
}
} // namespace dmotion