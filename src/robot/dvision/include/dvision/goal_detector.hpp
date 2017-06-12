/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:24+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: goal_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:23+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class GoalDetector : public IDetector
{
  public:
    explicit GoalDetector();
    ~GoalDetector();
    bool Init();

    bool GetPosts(cv::Mat& cannyImg,
                  cv::Mat& rawHSV,
                  cv::Mat& gray,
                  const cv::Mat& binaryFrame,
                  Projection& projection,
                  const std::vector<cv::Point>& fieldHull,
                  std::vector<LineSegment>& resLines,
                  std::vector<LineSegment>& alllLines,
                  std::vector<cv::Point2f>& goalPosition,
                  const bool& SHOWGUI,
                  cv::Mat& guiImg);

    bool checkDistance_Box(cv::Point2f downPointInReal, double length);
    bool checkDownPointDistance(const cv::Point& down, double& jumpDouble, Projection& projection, std::vector<cv::Point> fieldHull);
    bool voteGoalPostPoint(LineSegment& tmpLine,
                           const cv::Point2d& point,
                           const double& jumpDouble,
                           const bool& SHOWGUI,
                           cv::Mat& guiImg,
                           const cv::Mat& rawHSV,
                           const cv::Mat& cannyImg,
                           double& leftAvg,
                           double& rightAvg,
                           int& vote_for_doubleLeft,
                           int& vote_for_doubleRight);

  private:
};
} // namespace dvision
