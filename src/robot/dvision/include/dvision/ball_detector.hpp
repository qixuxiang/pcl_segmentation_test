/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-10T10:44:45+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: ball_detector.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-10T10:44:58+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "darknetcxx/detector.hpp"
#include "dvision/idetector.hpp"
#include <ros/ros.h>
#include <string>
#include <vector>

namespace dvision {
class BallDetector : public IDetector
{
  public:
    explicit BallDetector();
    ~BallDetector();
    bool Init();

    bool GetBall();

  private:
    darknet::Network* net_;
    std::vector<std::string> label_list_;
};
} // namespace dvision
