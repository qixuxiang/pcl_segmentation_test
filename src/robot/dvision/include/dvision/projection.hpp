// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// TODO(MWX):
// Version 1: only consider camera yaw and pitch

#pragma once
#include "dvision/distortionModel.hpp"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class Projection
{
  public:
    explicit Projection(ros::NodeHandle* nh);
    ~Projection();

  public:
    void update();
    void update(double yaw, double pitch);

    void getOnImageCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& resPoints);
    void getOnRealCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& resPoints);

    // single point
    void getOnImageCoordinate(const cv::Point& point, cv::Point2f& resPoint);
    void getOnRealCoordinate(const cv::Point2f& point, cv::Point& resPoint);

  private:
    void calcHomography();

  private:
    ros::NodeHandle* m_nh; // read parameters

    DistortionModel m_dist;
    cv::Size m_imageSize;
    cv::Mat m_distCoeff;
    cv::Mat m_cameraMatrix;

    // VERSION 1, use only yaw and pitch, 2017/5/29
    double m_yaw;
    double m_pitch;

    // VERSION 2, use TF

    cv::Point3d cameraLocation;
    cv::Point3d cameraOrientation;

    cv::Mat homoImgToReal; // real = homo * img
    cv::Mat homoReaToImg; // img = homo * real
};
} // namespace dvision
