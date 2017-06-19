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
    bool Process(cv::Mat& m_canny_img, cv::Mat& m_hsv_img, cv::Mat& m_gui_img, std::vector<cv::Point>& hull_field, Projection& m_projection);

    bool GetPosts(cv::Mat& canny_img,
                  cv::Mat& raw_hsv,
                  Projection& projection,
                  const std::vector<cv::Point>& field_hull,
                  std::vector<LineSegment>& res_lines,
                  std::vector<LineSegment>& all_lines,
                  const bool& SHOWGUI,
                  cv::Mat& gui_img);

    bool CheckDistanceBox(cv::Point2f down_point_in_real, double length);
    bool CheckDownPointDistance(const cv::Point& down, double& jump_double, Projection& projection, std::vector<cv::Point> field_hull);
    bool VoteGoalPostPoint(LineSegment& tmp_line,
                           const cv::Point2d& point,
                           const double& jump_double,
                           const bool& SHOWGUI,
                           cv::Mat& gui_img,
                           const cv::Mat& raw_hsv,
                           const cv::Mat& canny_img,
                           double& left_avg,
                           double& right_avg,
                           int& vote_for_double_left,
                           int& vote_for_double_right,
                           std::vector<cv::Point>& goal_post_points);
    inline std::vector<cv::Point2f> goal_position()
    {
        return goal_position_;
    }

  private:
    std::vector<cv::Point2f> goal_position_;
};
} // namespace dvision
