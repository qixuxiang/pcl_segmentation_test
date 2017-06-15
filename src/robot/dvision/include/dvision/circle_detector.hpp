/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:19:44+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: circle_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:02+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class CircleDetector : public IDetector
{
  public:
    explicit CircleDetector();
    ~CircleDetector();
    bool Init();
    bool Process(cv::Point2d& result_circle, std::vector<LineSegment>& clustered_lines);

    bool GetCircle(const double& H2, std::vector<LineSegment>& clustered_lines, cv::Point2d& result_circle);

  private:
};
} // namespace dvision
