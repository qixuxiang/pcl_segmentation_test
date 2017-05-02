#include "dmotion/dmotion.hpp"
#include <ros/ros.h>

namespace dmotion {
static const int MOTION_FREQ = 100;
DMotion::DMotion(ros::NodeHandle* nh) : DProcess(MOTION_FREQ, true), m_nh(nh) {

}

DMotion::~DMotion() {
}


void DMotion::tick() {
  ROS_INFO("motion tick");
}
}