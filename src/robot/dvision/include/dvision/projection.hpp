// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// TODO(MWX):
// Version 1: only consider camera yaw and pitch

#pragma once
#include "dvision/distortionModel.hpp"
#include "dvision/ipm.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {
class Projection
{
  public:
    explicit Projection(ros::NodeHandle* nh);
    ~Projection();

  public:
    bool update();
    bool update(double yaw, double pitch);
    bool calcHomography();

    bool getOnImageCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& resPoints);
    bool getOnRealCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& resPoints);

    // single point
    bool getOnImageCoordinate(const cv::Point& point, cv::Point2f& resPoint);
    bool getOnRealCoordinate(const cv::Point2f& point, cv::Point& resPoint);

  private:
    void init();

  private:
    ros::NodeHandle* m_nh; // read parameters

    DistortionModel m_dist;
    IPM m_ipm;
    cv::Size m_imageSize;
    cv::Mat m_distCoeff;
    cv::Mat m_cameraMatrix;

    // VERSION 1, use only yaw and pitch, 2017/5/29
    double m_yaw;
    double m_pitch;

    // VERSION 2, use TF

    cv::Point3d m_cameraLocation;
    cv::Point3d m_cameraOrientation;

    cv::Mat homoImgToReal; // real = homo * img
    cv::Mat homoReaToImg;  // img = homo * real
};
} // namespace dvision
