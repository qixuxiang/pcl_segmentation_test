// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"
#include "dconfig/dconstant.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle* n)
  : DProcess(VISION_FREQ, false)
  , m_nh(n)
{
    parameters.init(n);
    m_projection.init(n);
    // m_ball.Init();
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_loc.Init();

    m_concurrent.push([] {
        //     ROS_INFO("concurrent");
    });
    m_transmitter = new dtransmit::DTransmit(parameters.udpBroadcastAddress);
    m_sub_action_cmd = m_nh->subscribe("/humanoid/MotionInfo", 100, &DVision::motionCallback, this);
    m_sub_save_img = m_nh->subscribe("/humanoid/SaveImg", 100, &DVision::saveImgCallback, this);
    m_pub = m_nh->advertise<VisionInfo>("/humanoid/VisionInfo", 100);
}

DVision::~DVision()
{
}

void
DVision::tick()
{
    ROS_DEBUG("dvision tick");

    /**********
     * Update *
     **********/

    auto frame = m_camera.capture();
    VisionInfo m_data;

    m_projection.updateExtrinsic(m_pitch, m_yaw);

    if (!m_loc.Update(m_projection)) {
        ROS_ERROR("Cannot update localization!");
    }

    // get image in BGR and HSV color space
    m_gui_img = frame.getBGR_raw();
    m_hsv_img = frame.getHSV();

    /******************
     * Field Detector *
     ******************/

    m_data.see_field = m_field.Process(m_hsv_img, m_gui_img, m_projection);

    if (parameters.field.enable && !m_data.see_field) {
        ROS_ERROR("Detecting field failed.");
    }

    /*****************
     * Line Detector *
     *****************/

    m_data.see_line = m_line.Process(m_canny_img, m_hsv_img, m_gui_img, m_field.field_convex_hull(), m_field.field_binary_raw(), m_projection);

    if (!m_data.see_line) {
        ROS_ERROR("Detecting lines failed.");
    }

    /*******************
     * Circle detector *
     *******************/

    if (m_data.see_field && m_data.see_line) {
        m_data.see_circle = m_circle.Process(m_line.clustered_lines());
    }

    /*****************
     * Goal Detector *
     *****************/

    if (m_data.see_field) {
        m_data.see_goal = m_goal.Process(m_canny_img, m_hsv_img, m_gui_img, m_field.hull_field(), m_projection);
    }

    /****************
     * Localization *
     ****************/
    m_loc_img = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

    if (m_data.see_field && m_data.see_line) {
        m_data.loc_ok = m_loc.Calculate(m_line.clustered_lines(),
                                        m_data.see_circle,
                                        m_field.field_hull_real_center(),
                                        m_field.field_hull_real(),
                                        m_field.field_hull_real_rotated(),
                                        m_circle.result_circle(),
                                        m_goal.goal_position(),
                                        m_loc_img,
                                        m_projection);
    }

    /*****************
     * Ball Detector *
     *****************/

    // m_ball.GetBall(frame.getBGR_raw(), m_data, m_projection);

    /***********
     * Publish *
     ***********/

    prepareVisionInfo(m_data);
    m_pub.publish(m_data);
    m_transmitter->sendRos(dconstant::network::robotBroadcastAddressBase + parameters.robotId, m_data);

    // TODO(MWX): simulating mode

    /****************
     * Post process *
     ****************/

    showDebugImg();
    m_concurrent.spinOnce();
    m_concurrent.join();
}

void
DVision::motionCallback(const dmotion::MotionInfo::ConstPtr& motion_msg)
{
    m_motion_info = *motion_msg;
    m_pitch = static_cast<int>(m_motion_info.action.cmd_head.y);
    m_yaw = static_cast<int>(m_motion_info.action.cmd_head.z);
}

void
DVision::saveImgCallback(const SaveImg::ConstPtr& save_img_msg)
{
    m_save_img = *save_img_msg;
    if (m_save_img.IsSaveImg) {
        auto frame = m_camera.capture();
        std::string path_str;
        path_str = "p_" + std::to_string(m_pitch) + "_y_" + std::to_string(m_yaw) + " ";
        frame.save(path_str);
    }
}

void
DVision::prepareVisionInfo(VisionInfo& m_data)
{
    // localization
    m_data.robot_pos.x = m_loc.location().x;
    m_data.robot_pos.y = m_loc.location().y;
    m_data.robot_pos.z = m_loc.location().z;
    // goal
    // TODO(corenel) Get global coord?
    std::vector<cv::Point2f> goal_position_rotated = m_projection.RotateTowardHeading(m_goal.goal_position());
    m_data.see_goal = false;
    m_data.see_both_goal = false;
    m_data.see_unknown_goal = false;

    if (goal_position_rotated.size() >= 1) {
        m_data.see_goal = true;
        m_data.left_goal.x = goal_position_rotated[0].x;
        m_data.left_goal.y = goal_position_rotated[0].y;
        if (goal_position_rotated.size() == 2) {
            m_data.see_both_goal = true;
            m_data.right_goal.x = goal_position_rotated[1].x;
            m_data.right_goal.y = goal_position_rotated[1].y;
        } else if (goal_position_rotated.size() == 3) {
            m_data.see_unknown_goal = true;
            m_data.unknown_goal.x = goal_position_rotated[2].x;
            m_data.unknown_goal.y = goal_position_rotated[2].y;
        }
    }
    // circle
    cv::Point2d result_circle_rotated = m_projection.RotateTowardHeading(m_circle.result_circle());
    m_data.circle_field.x = result_circle_rotated.x;
    m_data.circle_field.y = result_circle_rotated.y;
    // ball
    // if (m_data.loc_ok) {
    //     // TODO(corenel) Rotate angle is correct?
    //     cv::Point2f ball_global;
    //     ball_global = m_projection.RotateTowardHeading(cv::Point2f(m_data.ball_field.x, m_data.ball_field.y));
    //     m_data.ball_global.x = ball_global.x;
    //     m_data.ball_global.y = ball_global.y;
    // }
}

void
DVision::showDebugImg()
{
    if (parameters.monitor.update_loc_img) {
        cv::namedWindow("loc", CV_WINDOW_NORMAL);
        cv::imshow("loc", m_loc_img);
    }

    if (parameters.monitor.update_gui_img) {
        cv::namedWindow("gui", CV_WINDOW_NORMAL);
        cv::imshow("gui", m_gui_img);
        // using for change hsv of white
        // cv::createTrackbar("h_low", "gui", &parameters.goal.h0, 255);
        // cv::createTrackbar("h_high", "gui", &parameters.goal.h1, 255);
        // cv::createTrackbar("s_low", "gui", &parameters.goal.s0, 255);
        // cv::createTrackbar("s_high", "gui", &parameters.goal.s1, 255);
        // cv::createTrackbar("v_low", "gui", &parameters.goal.v0, 255);
        // cv::createTrackbar("v_high", "gui", &parameters.goal.v1, 255);
        cv::createTrackbar("MinLineLength", "gui", &parameters.line.MinLineLength, 255);
        cv::createTrackbar("AngleToMerge", "gui", &parameters.line.AngleToMerge, 255);
        cv::createTrackbar("DistanceToMerge", "gui", &parameters.line.DistanceToMerge, 255);
        cv::createTrackbar("confiusedDist", "gui", &parameters.circle.confiusedDist, 255);

        cv::waitKey(1);
    }
}

} // namespace dvision
