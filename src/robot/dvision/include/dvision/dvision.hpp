// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dvision/camera.hpp"
#include "dvision/circle_detector.hpp"
#include "dvision/field_detector.hpp"
#include "dvision/goal_detector.hpp"
#include "dvision/line_detector.hpp"
#include "dvision/localization.hpp"
#include "dvision/projection.hpp"
#include <vector>

namespace dvision {
class DVision : public dprocess::DProcess<DVision>
{
  public:
    explicit DVision(ros::NodeHandle* n);
    ~DVision();
    void tick() override;

  private:
    ros::NodeHandle* m_nh;
    Camera m_camera;
    Projection m_projection;
    dprocess::DConcurrent m_concurrent;

    // detectors and loc
    CircleDetector m_circle;
    LineDetector m_line;
    GoalDetector m_goal;
    FieldDetector m_field;
    Localization m_loc;

    // field hull
    std::vector<cv::Point2f> m_field_hull_real;
    std::vector<cv::Point2f> m_field_hull_real_rotated;
    cv::Point2f m_field_hull_real_center;

    // img
    cv::Mat m_raw_hsv_img, m_gray_img;
    cv::Mat m_gui_top_view_rotate, m_gui_img, m_gui_undist;
    cv::Mat m_ball_binary, m_field_binary, m_goal_binary;
    cv::Mat m_field_convect_hull, m_canny_img_in_field;
};
} // namespace dvision
