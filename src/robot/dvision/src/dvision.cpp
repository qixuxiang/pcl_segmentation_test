// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
#include "dvision/dvision.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle *n) : DProcess(VISION_FREQ, false), m_nh(n) {

}

DVision::~DVision() {

}

void DVision::tick() {
  ROS_INFO("dvision tick");

  // x.spinOnce();
  // y.spinOnce();
  // x.join();
  // y.join();
}
}
