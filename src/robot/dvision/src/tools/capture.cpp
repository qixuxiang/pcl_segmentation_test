#include "dvision/camera.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "capture");
  ros::NodeHandle nh;

  dvision::Camera c;
  while(ros::ok()) {
    auto frame = c.capture();
    frame.show();
  }
}