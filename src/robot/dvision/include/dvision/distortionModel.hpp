// Copied from Nimbro
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class DistortionModel
{
  public:
    explicit DistortionModel(cv::Size imageSize, cv::Mat cameraMatrix, cv::Mat distCoeff);
    void undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);
    void undistortImage(const cv::Mat& src, cv::Mat& dst);

  private:
    void init();
    void undistort_slow(const std::vector<cv::Point>& points, std::vector<cv::Point>& resPoints);

  private:
    std::vector<cv::Point> m_distortionVector;
    cv::Size m_imageSize;
    cv::Size m_undistImageSize;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeff;
};
} // namespace dvision
