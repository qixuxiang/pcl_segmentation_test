// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dmotion/ActionCmd.h"
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dvision/SaveImg.h"
#include "dvision/ball_detector.hpp"
#include "dvision/camera.hpp"
#include "dvision/circle_detector.hpp"
#include "dvision/field_detector.hpp"
#include "dvision/goal_detector.hpp"
#include "dvision/line_detector.hpp"
#include "dvision/localization.hpp"
#include "dvision/projection.hpp"
#include "dvision/utils.hpp"
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
    ros::Subscriber m_sub_action_cmd;
    ros::Subscriber m_sub_save_img;
    Camera m_camera;
    Projection m_projection;
    dprocess::DConcurrent m_concurrent;

    // detectors and loc
    BallDetector m_ball;
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
    cv::Mat m_hsv_img, m_gray_img, m_canny_img, m_gui_img;
    // cv::Mat m_gui_top_view_rotate, m_gui_img, m_gui_undist;
    cv::Mat m_ball_binary, m_field_binary, m_goal_binary;
    cv::Mat m_field_convex_hull;
    // added by yyj
    int m_yaw;
    int m_pitch;
    dmotion::ActionCmd m_action_cmd;
    SaveImg m_save_img;
    void motionCallback(const dmotion::ActionCmd::ConstPtr& msg);
    void saveImgCallback(const SaveImg::ConstPtr& save_img_msg);
};
} // namespace dvision
