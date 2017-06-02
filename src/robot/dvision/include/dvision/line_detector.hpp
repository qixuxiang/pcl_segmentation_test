/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:33+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: line_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:25+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include <ros/ros.h>
#include <vector>

namespace dvision {
class LineDetector
{
  public:
    explicit LineDetector();
    ~LineDetector();
    void init();

  private:
};
} // namespace dvision
