// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>


#include "dvision/frame.hpp"
#include <opencv2/opencv.hpp>
#include <string>

namespace dvision {
void Frame::save(std::string path) {
  cvt();
  cv::imwrite(path + std::to_string(m_timeStamp.toNSec()) + ".png", m_rgb);
}

void callback(int state, void* userdata) {
  Frame* f = static_cast<Frame*>(userdata);
  f->save("./");
}

static bool button_created = false;

void Frame::show() {
  cvt();
  cv::namedWindow("camera", CV_WINDOW_NORMAL);
  if (!button_created) {
    cv::createButton("save", callback, this, CV_PUSH_BUTTON);
    button_created = true;
  }
  cv::imshow("camera", m_rgb);
  cv::waitKey(1);
}
}  // namespace dvision
