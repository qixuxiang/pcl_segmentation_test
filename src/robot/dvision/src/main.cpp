// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include <ros/ros.h>
#include "dvision/dvision.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dmotion_node");
  ros::NodeHandle nh;
  dvision::DVision v(&nh);
  v.spin();
  v.join();
}
