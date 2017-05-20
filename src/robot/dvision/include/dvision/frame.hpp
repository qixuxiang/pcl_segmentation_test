#pragma once
#include <ros/ros.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace dvision {
class Frame {
public:
  inline explicit Frame(uint8_t *yuv, int width, int height) : m_yuv(yuv), m_width(width), m_height(height), m_converted(false) {
    m_timeStamp = ros::Time::now();
  }

  inline ~Frame() {};

  inline cv::Mat& getRGB() { return m_rgb; }

  inline void cvt() {
    if(m_converted) return;
    cv::Mat yuvMat(m_height, m_width, CV_8UC2, m_yuv);
    cv::cvtColor(yuvMat, m_rgb, CV_YUV2BGR_YUYV);
    m_converted = true;
  }

  void show();

  void save(std::string path);
private:
  uint8_t *m_yuv; // raw yuv image
  cv::Mat m_rgb;
  ros::Time m_timeStamp;
  int m_width;
  int m_height;
  bool m_converted;
};
} // namespace dvision