// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class Projection
{
  public:
    Projection();
    ~Projection();

  public:
    void update();
    void getOnImageCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& resPoints);
    void getOnRealCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& resPoints);

    // single point
    void getOnImageCoordinate(const cv::Point& point, cv::Point2f& resPoint);
    void getOnRealCoordinate(const cv::Point2f& point, cv::Point& resPoint);

  private:
    void init();

  private:
    cv::Mat m_distCoeff;
    cv::Mat m_cameraMatrix;
    cv::Point3d cameraLocation;
    cv::Point3d cameraOrientation;

    cv::Mat homoImgToReal; // real = homo * img
    cv::Mat homoReaToImg; // img = homo * real
};
} // namespace dvision
