/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:24+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: goal_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:23+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class GoalDetector : public IDetector
{
  public:
    explicit GoalDetector();
    ~GoalDetector();
    void init();

  private:
};
} // namespace dvision
