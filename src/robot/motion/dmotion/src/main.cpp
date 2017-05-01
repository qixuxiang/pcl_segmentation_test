#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dmotion_node");
  ros::NodeHandle n;
  ros::Rate r(1);
  while(ros::ok()) {
    ROS_INFO("Hi ROS");
    r.sleep();
  }
}
