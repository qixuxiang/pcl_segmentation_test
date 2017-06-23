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
#include "dvision/VisionInfo.h"
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

    bool GetBall(const cv::Mat& frame, cv::Mat& gui_img, Projection& m_projection);
    bool CvtRelativePosition(std::vector<darknet::RelateiveBBox>& ball_position, std::vector<darknet::bbox>& ball_position_cvt);
    inline cv::Point ball_image()
    {
        return ball_image_;
    }
    inline cv::Point2f ball_field()
    {
        return ball_field_;
    }

  private:
    darknet::Network* net_;
    std::vector<std::string> label_list_;
    darknet::Image raw_img_;
    cv::Point ball_image_;
    cv::Point ball_image_top_;
    cv::Point ball_image_bottom_;
    cv::Point2f ball_field_;
};
} // namespace dvision
