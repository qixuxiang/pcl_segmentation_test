// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle* n)
  : DProcess(VISION_FREQ, false)
  , m_nh(n)
{
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
    m_sub_action_cmd = m_nh->subscribe("/humanoid/MotionFeedback", 1, &DVision::motionCallback, this);
    m_sub_save_img = m_nh->subscribe("/humanoid/SaveImg", 1, &DVision::saveImgCallback, this);
    m_pub = m_nh->advertise<VisionShareData>("/humanoid/VisionShareData", 1);
}

DVision::~DVision()
{
}

void
DVision::tick()
{
    ROS_DEBUG("dvision tick");
    auto frame = m_camera.capture();

    /**********
     * Update *
     **********/
    m_projection.updateExtrinsic(m_pitch, m_yaw);

    if (!m_loc.Update(m_projection)) {
        ROS_ERROR("Cannot update localization!");
    }

    // get image in BGR and HSV color space
    m_gui_img = frame.getBGR_raw();
    cv::cvtColor(m_gui_img, m_hsv_img, CV_BGR2HSV);

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
    if (m_data.see_field && m_data.see_line && m_data.see_circle || m_data.see_goal) {
        m_data.loc_ok = m_loc.Calculate(m_line.clustered_lines(),
                                        m_data.see_circle,
                                        m_field.field_hull_real_center(),
                                        m_field.field_hull_real(),
                                        m_field.field_hull_real_rotated(),
                                        m_circle.result_circle(),
                                        m_goal.goal_position(),
                                        m_projection);
    }

    /*****************
     * Ball Detector *
     *****************/

    // m_ball.GetBall(frame.getBGR_raw(), m_data, m_projection);

    /***********
     * Publish *
     ***********/

    prepareVisionShareData();
    m_pub.publish(m_data);

    /****************
     * Post process *
     ****************/

    cv::namedWindow("gui_img", CV_WINDOW_NORMAL);
    cv::imshow("gui_img", m_gui_img);
    cv::waitKey(1);

    m_concurrent.spinOnce();
    m_concurrent.join();
}

void
DVision::motionCallback(const dmotion::ActionCmd::ConstPtr& motion_msg)
{
    m_action_cmd = *motion_msg;
    m_pitch = static_cast<int>(m_action_cmd.cmd_head.y);
    m_yaw = static_cast<int>(m_action_cmd.cmd_head.z);
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
DVision::prepareVisionShareData()
{
    // localization
    m_data.robot_pos.x = m_loc.location().x;
    m_data.robot_pos.y = m_loc.location().y;
    m_data.robot_pos.z = m_loc.location().z;
    // goal
    // TODO(corenel) Get global coord?
    std::vector<cv::Point2f> goal_position = m_goal.goal_position();
    if (goal_position.size() >= 1) {
        m_data.see_goal = true;
        m_data.left_goal.x = goal_position[0].x;
        m_data.left_goal.y = goal_position[0].y;
        if (goal_position.size() == 2) {
            m_data.see_both_goal = true;
            m_data.right_goal.x = goal_position[1].x;
            m_data.right_goal.y = goal_position[1].y;
        } else if (goal_position.size() == 3) {
            m_data.see_unknown_goal = true;
            m_data.unknown_goal.x = goal_position[2].x;
            m_data.unknown_goal.y = goal_position[2].y;
        }
    }
    // ball
    // if (m_data.loc_ok) {
    //     // TODO(corenel) Rotate angle is correct?
    //     cv::Point2f ball_global;
    //     ball_global = m_projection.RotateTowardHeading(cv::Point2f(m_data.ball_field.x, m_data.ball_field.y));
    //     ball_global += cv::Point2f(m_data.robot_pos.x, m_data.robot_pos.y);
    //     m_data.ball_global.x = ball_global.x;
    //     m_data.ball_global.y = ball_global.y;
    // }
}

} // namespace dvision
