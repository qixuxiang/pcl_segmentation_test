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
    ROS_INFO("dvision tick");
    auto frame = m_camera.capture();

    /**********
     * Update *
     **********/
    // TODO(mwx) get yaw and pitch from motor
    double pitch = 0;
    double yaw = 0;

    if (!m_loc.Update(m_projection)) {
        ROS_ERROR("Cannot update localization!");
    }

    // get image in BGR and HSV color space
    m_gui_img = frame.getBGR();
    cv::cvtColor(m_gui_img, m_hsv_img, CV_BGR2HSV);

    /******************
     * Field Detector *
     ******************/
    bool field_detection_OK = false;

    // TODO(corenel) move it ro m_field
    cv::Mat field_binary_raw;
    std::vector<cv::Point> hull_field;

    field_detection_OK =
      m_field.Process(m_field_hull_real, m_field_hull_real_rotated, hull_field, m_field_binary, field_binary_raw, m_field_convex_hull, m_hsv_img, m_gui_img, m_field_hull_real_center, m_projection);

    if (parameters.field.enable && !field_detection_OK) {
        ROS_ERROR("Detecting field failed.");
    }

    /*****************
     * Line Detector *
     *****************/

    // TODO(corenel) move it ro m_line
    std::vector<LineSegment> clustered_lines;

    if (!m_line.Process(m_canny_img, m_hsv_img, m_gui_img, m_field_convex_hull, field_binary_raw, clustered_lines, m_projection)) {
        ROS_ERROR("Detecting lines failed.");
    }

    /*******************
     * Circle detector *
     *******************/

    // TODO(corenel) move it ro m_circle
    cv::Point2d result_circle;

    if (field_detection_OK) {
        m_data.see_circle = m_circle.Process(result_circle, clustered_lines);
    }

    /*****************
     * Goal Detector *
     *****************/

    if (field_detection_OK) {
        m_data.see_goal = m_goal.Process(m_canny_img, m_hsv_img, m_gui_img, hull_field, m_projection);
    }

    /****************
     * Localization *
     ****************/

    m_data.loc_ok = m_loc.Calculate(clustered_lines, m_data.see_circle, m_field_hull_real_center, m_field_hull_real, m_field_hull_real_rotated, result_circle, m_goal.goal_position(), m_projection);

    /*****************
     * Ball Detector *
     *****************/

    // m_ball.GetBall(frame.getBGR(), m_data, m_projection);

    /***********
     * Publish *
     ***********/

    prepareVisionShareData();
    m_pub.publish(m_data);

    /****************
     * Post process *
     ****************/

    m_concurrent.spinOnce();
    m_concurrent.join();
}

void
DVision::motionCallback(const dmotion::ActionCmd::ConstPtr& motion_msg)
{
    m_action_cmd = *motion_msg;
    std::cout << "fuck: " << m_action_cmd.cmd_head.y << " " << m_action_cmd.cmd_head.z << std::endl;
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
        } else {
            m_data.see_unknown_goal = true;
            m_data.unknown_goal.x = goal_position[1].x;
            m_data.unknown_goal.y = goal_position[1].y;
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
