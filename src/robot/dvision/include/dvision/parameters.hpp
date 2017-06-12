#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>

namespace dvision {

struct CameraParameters
{
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeff;

    cv::Size undistImageSize;
    cv::Mat undistCameraMatrix;
    cv::Mat undistDistCoeff;

    double fx;
    double fy;
    double cx;
    double cy;

    double undistCx;
    double undistCy;

    int width;
    int height;

    int undistWidth;
    int undistHeight;

    // TODO(MWX): Extrinsic parameters
};

struct Parameters
{
    CameraParameters camera;

    void init(ros::NodeHandle* nh);
};

extern Parameters parameters;
}