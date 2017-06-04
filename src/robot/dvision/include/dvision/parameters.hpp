#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

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

struct CircleDetectorParameters
{

}

struct FieldDetectorParameters
{

}

struct GoalDetectorParameters
{

}

struct LineDetectorParameters
{

}

struct LocalizationParameters
{

}

struct Parameters
{
    CameraParameters camera;

    void init(ros::NodeHandle* nh);
};

extern Parameters parameters;
}
