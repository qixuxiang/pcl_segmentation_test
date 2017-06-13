/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:33+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: line_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:25+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class LineDetector : public IDetector
{
  public:
    explicit LineDetector();
    ~LineDetector();
    bool Init();
    bool Process(cv::Mat& m_canny_img,
                 cv::Mat& m_hsv_img,
                 cv::Mat& m_gui_img,
                 cv::Mat& m_canny_img_in_field,
                 cv::Mat& m_field_convex_hull,
                 cv::Mat& field_binary_raw,
                 std::vector<LineSegment>& clustered_lines,
                 Projection& m_projection);

    // bool
    // GetLines(cv::Mat& rawHSV, cv::Mat& fieldMask, cv::Mat& guiImg, Projection& projections, const bool& showGui, const cv::Mat& lineBinary, const cv::Rect& box, std::vector<LineSegment>& resLines);
    bool GetLines(cv::Mat& raw_hsv, cv::Mat& field_mask, cv::Mat& gui_img, const bool& SHOWGUI, const cv::Mat& line_binary, std::vector<LineSegment>& res_lines);

  private:
};
} // namespace dvision
