/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:41+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: field_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:27+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include <ros/ros.h>
#include <vector>

namespace dvision {
class FieldDetector
{
  public:
    explicit FieldDetector();
    ~FieldDetector();
    void init();

  private:
};
} // namespace dvision
