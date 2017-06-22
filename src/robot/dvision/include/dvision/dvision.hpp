// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dbehavior/BehaviourInfo.h"
#include "dmotion/ActionCommand.h"
#include "dmotion/MotionInfo.h"
#include "dmotion/GetDelta.h"
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dvision/SaveImg.h"
#include "dvision/VisionInfo.h"
#include "dvision/ball_detector.hpp"
#include "dvision/ball_tracker.hpp"
#include "dvision/camera.hpp"
#include "dvision/circle_detector.hpp"
#include "dvision/field_detector.hpp"
#include "dvision/goal_detector.hpp"
#include "dvision/line_detector.hpp"
#include "dvision/localization.hpp"
#include "dvision/projection.hpp"
#include "dvision/utils.hpp"
#include "std_msgs/String.h"
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
    ros::ServiceClient m_deltaClient;
    ros::Subscriber m_sub_motion_info;
    ros::Subscriber m_sub_behaviour_info;
    ros::Subscriber m_sub_reload_config;
    ros::Publisher m_pub;
    Camera* m_camera;
    Projection m_projection;
    dprocess::DConcurrent m_concurrent;
    dtransmit::DTransmit* m_transmitter;
    // todo add broadcast address config

    // detectors and loc
    BallDetector m_ball;
    CircleDetector m_circle;
    LineDetector m_line;
    GoalDetector m_goal;
    FieldDetector m_field;
    Localization m_loc;

    // image
    cv::Mat m_hsv_img;
    cv::Mat m_gui_img, m_loc_img, m_canny_img;
    cv::Mat m_field_binary, m_goal_binary, m_line_binary;
    // added by yyj
    BalllTracker m_ball_tracker;
    int m_center_pitch;
    int m_center_yaw;
    int m_pitch = 0;
    int m_yaw = 0;

    VisionInfo m_data;
    dmotion::MotionInfo m_motion_info;
    dbehavior::BehaviourInfo m_behaviour_info;
    void motionCallback(const dmotion::MotionInfo::ConstPtr& msg);
    void behaviourCallback(const dbehavior::BehaviourInfo::ConstPtr& msg);
    void reloadConfigCallback(const std_msgs::String::ConstPtr&);
    void prepareVisionInfo(VisionInfo& m_data);
    void showDebugImg();
    void updateViewRange();
};
} // namespace dvision
