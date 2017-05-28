// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <opencv2/opencv.hpp>

namespace dvision {
class Projection
{
    using namespace cv;

  public:
    Projection();

  private:
    Mat m_distCoeff;
    Mat m_cameraMatrix;
    bool initialised = false;
};
} // namespace dvision
