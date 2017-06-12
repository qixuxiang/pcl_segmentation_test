/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:41+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: field_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:27+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class FieldDetector : public IDetector
{
  public:
    explicit FieldDetector();
    ~FieldDetector();
    bool Init();

    bool GetPoints(cv::Mat& binaryFrame, std::vector<cv::Point>& resPoints, std::vector<std::vector<cv::Point>>& allFieldContours);
    void FindInField(const cv::Mat& srcHsvImg, const cv::Mat& templateGrayImg, cv::Mat* dstGrayImgs, HSVRange* ranges, bool* inTemplate, int size = 1);
    std::vector<cv::Point> getBodyMaskContourInRaw(float rot);

    std::vector<cv::Point> BodyMaskContourInverted;

  private:
};
} // namespace dvision
