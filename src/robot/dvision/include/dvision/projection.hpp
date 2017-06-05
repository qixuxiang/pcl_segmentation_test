// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
// TODO(MWX):
// Version 1: only consider camera yaw and pitch

#pragma once
#include "dvision/distortionModel.hpp"
#include "dvision/ipm.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {
class Projection
{
  public:
    explicit Projection();
    ~Projection();
    void init(ros::NodeHandle*);

  public:
    bool updateExtrinsic(double yaw, double pitch);

    inline bool calcHomography()
    {
        return m_ipm.initGetHomography(m_extrinsic, homoImgToReal, homoRealToImg);
    }

    bool getOnImageCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& resPoints);
    bool getOnRealCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& resPoints);

    // single point
    bool getOnImageCoordinate(const cv::Point2f& point, cv::Point& resPoint);
    bool getOnRealCoordinate(const cv::Point& point, cv::Point2f& resPoint);

  private:
    void init();

  private:
    DistortionModel m_dist;
    IPM m_ipm;
    // VERSION 1, use only yaw and pitch, 2017/5/29
    double m_yaw;
    double m_pitch;
    Eigen::MatrixXd m_extrinsic;

    cv::Mat homoImgToReal; // real = homo * img
    cv::Mat homoRealToImg; // img = homo * real
};
} // namespace dvision
