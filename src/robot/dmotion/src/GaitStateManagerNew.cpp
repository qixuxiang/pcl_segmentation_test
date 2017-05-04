#include "dmotion/GaitStateManager.hpp"
#include <ros/ros.h>

namespace dmotion {
GaitStateManager::GaitStateManager() {}

GaitStateManager::~GaitStateManager() {}

void GaitStateManager::checkNewCommand(const ActionCmd &cmd) {
  auto& vel = cmd.cmd_vel;
  auto x = vel.linear.x;
  auto y = vel.linear.y;
  auto t = vel.angular.z;
  ROS_INFO("CMD %lf %lf %lf", x, y, t);

}

void GaitStateManager::tick() {
//  ROS_INFO("GaitStateManager tick");
}
}