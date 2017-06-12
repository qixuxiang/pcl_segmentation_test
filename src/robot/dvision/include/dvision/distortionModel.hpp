// Copied from Nimbro
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class DistortionModel
{
  public:
    DistortionModel();
    void init();

    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);
    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res);
    void undistortImage(const cv::Mat& src, cv::Mat& dst);
    void undistortImage2(const cv::Mat& src, cv::Mat& dst);

  private:
    bool undistort_slow(const std::vector<cv::Point>& points, std::vector<cv::Point>& resPoints);

  private:
    std::vector<cv::Point> m_distortionVector;
    cv::Mat m_map1;
    cv::Mat m_map2;
};
} // namespace dvision
