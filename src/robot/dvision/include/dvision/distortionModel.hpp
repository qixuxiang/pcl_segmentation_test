// Copied from Nimbro
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace dvision {
class DistortionModel
{
  public:
    DistortionModel();
    explicit DistortionModel(cv::Size imageSize, cv::Mat cameraMatrix, cv::Mat distCoeff);
    void init(cv::Size imageSize, cv::Mat cameraMatrix, cv::Mat distCoeff);

    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point>& res);
    bool undistort(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& res);
    void undistortImage(const cv::Mat& src, cv::Mat& dst);
    void undistortImage2(const cv::Mat& src, cv::Mat& dst);

  public:

    inline cv::Size getUndistImageSize()
    {
        return m_undistImageSize;
    }

    inline cv::Mat getUndistCameraMatrix()
    {
        return m_undistCameraMatrix;
    }

  private:
    bool undistort_slow(const std::vector<cv::Point>& points, std::vector<cv::Point>& resPoints);

  private:
    std::vector<cv::Point> m_distortionVector;
    cv::Size m_imageSize;
    cv::Size m_undistImageSize;

    cv::Mat m_cameraMatrix;
    cv::Mat m_undistCameraMatrix; // optical center shifted
    cv::Mat m_distCoeff;

    cv::Mat m_map1;
    cv::Mat m_map2;
};
} // namespace dvision
