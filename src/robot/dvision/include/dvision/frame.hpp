#pragma once
#include <ros/ros.h>
#include <string>

namespace dvision {
class Frame {
public:
  inline explicit Frame(const uint8_t *yuv) : m_yuv(yuv) {
    m_timeStamp = ros::Time::now();
  }
  inline ~Frame() {};
  inline const uint8_t *get() { return m_yuv; }
  void save(std::string path);
  //TODO(MWX): get cvMat
private:
  const uint8_t *m_yuv; // raw yuv image
  ros::Time m_timeStamp;
};
} // namespace dvision