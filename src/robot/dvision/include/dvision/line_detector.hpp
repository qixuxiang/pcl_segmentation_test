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
// #include "dvision/projection.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class LineDetector : public IDetector
{
  public:
    explicit LineDetector();
    ~LineDetector();
    bool Init();

    // bool
    // GetLines(cv::Mat& rawHSV, cv::Mat& fieldMask, cv::Mat& guiImg, Projection& projections, const bool& showGui, const cv::Mat& lineBinary, const cv::Rect& box, std::vector<LineSegment>& resLines);
    bool GetLines(cv::Mat& rawHSV, cv::Mat& fieldMask, cv::Mat& guiImg, const bool& showGui, const cv::Mat& lineBinary, std::vector<LineSegment>& resLines);

  private:
};
} // namespace dvision
