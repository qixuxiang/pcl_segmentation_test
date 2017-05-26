// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include <opencv2/opencv.hpp>
#include "dvision/camera.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "capture");
  ros::NodeHandle nh;

  dvision::Camera c;
  while (ros::ok()) {
    auto frame = c.capture();
    frame.show();
  }
}
