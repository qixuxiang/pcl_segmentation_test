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
    m_ball.Init();
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_loc.Init();

    m_concurrent.push([] {
        //     ROS_INFO("concurrent");
    });
    m_sub_action_cmd = m_nh->subscribe("/humanoid/ActionCommand", 1, &DVision::motionCallback, this);
    m_sub_save_img = m_nh->subscribe("/humanoid/SaveImg", 1, &DVision::saveImgCallback, this);
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
    m_projection.updateExtrinsic(yaw, pitch);
    m_projection.calcHomography();

    if (!m_projection.updateExtrinsic(yaw, pitch)) {
        ROS_ERROR("Cannot update extrinsic of camera!");
    }

    if (!m_projection.calcHomography()) {
        ROS_ERROR("Cannot calculate homography!");
    }
    if (!m_loc.Update(m_projection)) {
        ROS_ERROR("Cannot update localization!");
    }

    // get image in BGR and HSV color space
    m_gui_img = frame.getRGB();
    cv::cvtColor(m_gui_img, m_hsv_img, CV_BGR2HSV);

    /******************
     * Field Detector *
     ******************/
    bool field_detection_OK = false;

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

    bool line_detection_OK = false;

    // calculate canny image
    std::vector<LineSegment> clustered_lines;

    line_detection_OK = m_line.Process(m_canny_img, m_hsv_img, m_gui_img, m_canny_img_in_field, m_field_convex_hull, field_binary_raw, clustered_lines, m_projection);

    if (parameters.line.enable && !line_detection_OK) {
        ROS_ERROR("Detecting lines failed.");
    }

    /*******************
     * Circle detector *
     *******************/

    bool circle_detected = false;
    bool confused = false;
    cv::Point2d result_circle;

    if (field_detection_OK) {
        circle_detected = m_circle.Process(confused, result_circle, clustered_lines);
    }

    /*****************
     * Goal Detector *
     *****************/

    bool goal_detection_OK = false;

    std::vector<cv::Point2f> goal_position_real;

    if (field_detection_OK) {
        goal_detection_OK = m_goal.Process(goal_position_real, m_canny_img, m_hsv_img, m_gui_img, m_gray_img, m_goal_binary, hull_field, m_projection);
    }

    /****************
     * Localization *
     ****************/
    bool loc_detection_OK = false;

    std::vector<LineContainer> all_lines;
    std::vector<FeatureContainer> all_features;

    if (parameters.loc.enable) {
        if (m_loc.Calculate(
              clustered_lines, circle_detected, m_field_hull_real_center, m_field_hull_real, m_field_hull_real_rotated, result_circle, goal_position_real, all_lines, all_features, m_projection)) {
            loc_detection_OK = true;
        }
    }

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

} // namespace dvision
