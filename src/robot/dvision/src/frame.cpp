#include "dvision/frame.hpp"
#include <opencv2/opencv.hpp>

namespace dvision {
void Frame::save(std::string path) {
  cvt();
  cv::imwrite(path, m_rgb);
}

void Frame::show() {
  cvt();
  cv::imshow("camera", m_rgb);
  cv::waitKey(1);
}
}