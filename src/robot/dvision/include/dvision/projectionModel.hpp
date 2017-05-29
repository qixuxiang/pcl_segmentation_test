// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <opencv2/opencv.hpp>

namespace dvision {
class ProjectionModel
{
  public:
    static void Init(cv::InputArray distCoeff, cv::InputArray cameraMatrix);
    static cv::Point2d image2world(Pixel); // TODO(MWX) robot gesture
    static Pixel world2image(World, RobotGesture);

  private:
    cv::Mat m_distortionCoefficient;
    cv::Mat m_cameraMatrix;
    bool initialised;
};
} // namespace dvision
