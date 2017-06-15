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
#include "dvision/VisionShareData.h"
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
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

    bool GetBall(cv::Mat& frame, VisionShareData& m_data, Projection& m_projection);
    bool CvtRelativePosition(std::vector<darknet::RelateiveBBox>& ball_position, std::vector<darknet::bbox>& ball_position_cvt);

  private:
    darknet::Network* net_;
    std::vector<std::string> label_list_;
};
} // namespace dvision
