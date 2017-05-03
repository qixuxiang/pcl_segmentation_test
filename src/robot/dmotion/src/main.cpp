#include <ros/ros.h>
#include "dmotion/dmotion.hpp"

using namespace dmotion;
int main(int argc, char** argv) {
  ros::init(argc, argv, "dmotion_node");
  ros::NodeHandle nh;

  DMotion d(&nh);
  d.spin();
  d.join();
}
