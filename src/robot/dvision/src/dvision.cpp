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
    m_sub_action_cmd = m_nh->subscribe("/humanoid/MotionFeedback", 1, &DVision::motionCallback, this);
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

    // clear field convex hull
    m_field_hull_real.clear();
    m_field_hull_real_rotated.clear();

    // create binary mask of field
    m_field_binary = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
    cv::Mat field_binary_raw = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
    cv::inRange(m_hsv_img, cv::Scalar(parameters.field.h0, parameters.field.s0, parameters.field.v0), cv::Scalar(parameters.field.h1, parameters.field.s1, parameters.field.v1), field_binary_raw);
    m_field_binary = field_binary_raw.clone();

    // draw field mask on gui image
    if (parameters.field.showMask && parameters.monitor.update_gui_img) {
        cv::Mat darker = cv::Mat::zeros(m_gui_img.size(), CV_8UC3);
        darker.copyTo(m_gui_img, 255 - field_binary_raw);
    }

    std::vector<cv::Point> field_points;
    std::vector<cv::Point> field_contour_undist;
    std::vector<std::vector<cv::Point>> all_field_contours;
    std::vector<cv::Point> hull_undist, hull_undist_mid_point, hull_field;

    // detect field
    if (parameters.field.enable && m_field.GetPoints(m_field_binary, field_points, all_field_contours)) {
        // show all field contours for debugging
        if (parameters.field.showDebug && parameters.monitor.update_gui_img) {
            cv::drawContours(m_gui_img, all_field_contours, -1, cv::Scalar(70, 220, 70), 2, 8);
        }

        // 计算非畸变下的轮廓点
        if (m_projection.undistort(field_points, field_contour_undist)) {
            // 计算非畸变下的凸包
            cv::convexHull(field_contour_undist, hull_undist, false);

            const int NUM_MID_P = 3;
            const int COUNT_MID_P = static_cast<int>(pow(2, NUM_MID_P) + 1);
            hull_undist_mid_point.reserve(COUNT_MID_P * hull_undist.size());
            std::vector<cv::Point2f> undist_point_pool, real_point_pool;
            undist_point_pool.resize(COUNT_MID_P * hull_undist.size());

            // 获得凸包的边缘线
            for (size_t i = 0; i < hull_undist.size(); i++) {
                size_t cur = i;
                size_t next = (i >= hull_undist.size() - 1) ? 0 : i + 1;
                LineSegment ls(hull_undist[cur], hull_undist[next]);
                // 把边缘线2^3+1等分,并把这些点存在undist_point_pool里面
                std::vector<cv::Point2d> result_mid_point = ls.GetMidPoints(NUM_MID_P, true);
                for (size_t j = 0; j < COUNT_MID_P; j++) {
                    undist_point_pool[i * COUNT_MID_P + j] = result_mid_point[j];
                }
            }

            // 如果x可以接受,距离原点(0,0)的距离可以接受,则把畸变后的点存在undist_point_pool中
            for (size_t i = 0; i < undist_point_pool.size(); i++) {
                hull_undist_mid_point.push_back(cv::Point(static_cast<int>(undist_point_pool[i].x), static_cast<int>(undist_point_pool[i].y)));
            }

            if (m_projection.undistort(hull_undist_mid_point, hull_field)) {
                // debug 模式下显示凸包边界点
                if (parameters.field.showDebug && parameters.monitor.update_gui_img) {
                    for (size_t i = 0; i < hull_field.size(); i++) {
                        cv::circle(m_gui_img, hull_field[i], 4, redColor(), 3);
                    }
                }
                std::vector<std::vector<cv::Point>> hulls(1, hull_field);
                m_field_convex_hull = cv::Mat::zeros(m_field_binary.size(), CV_8UC1);

                // 在m_field_convex_hull中划出凸包
                // grayWhite 就是灰度图下的255
                drawContours(m_field_convex_hull, hulls, -1, grayWhite(), CV_FILLED, 8);

                // 是否考虑body mask
                std::vector<cv::Point> tmp_body_mask_contour = m_field.GetBodyMaskContourInRaw(-Radian2Degree(0));
                bool consider_body_mask = (tmp_body_mask_contour.size() >= 6);

                if (consider_body_mask) {
                    cv::Mat body_mask_mat = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
                    std::vector<std::vector<cv::Point>> hullstmp_body_mask_contour(1, tmp_body_mask_contour);
                    drawContours(body_mask_mat, hullstmp_body_mask_contour, -1, grayWhite(), CV_FILLED, 8);
                    m_field_convex_hull -= body_mask_mat;
                }

                if (hull_field.size() > 1) {
                    std::vector<cv::Point2f> real_hull_points;
                    // 把场地边界转换为世界坐标系中的点
                    if (!m_projection.getOnRealCoordinate(hull_field, real_hull_points)) {
                        ROS_ERROR("Cannot get real coord of hull field");
                    }

                    m_field_hull_real_center.x = 0;
                    m_field_hull_real_center.y = 0;
                    m_field_hull_real.resize(real_hull_points.size());

                    // 把世界做坐标系中的点存在了m_field_hull_real中,并求所有点的x,y的均值存在m_field_hull_real_center中
                    // TODO(yyj) 感觉没有必要定义real_hull_points
                    for (size_t fI = 0; fI < real_hull_points.size(); fI++) {
                        m_field_hull_real[fI] = real_hull_points[fI];
                        m_field_hull_real_center.x += real_hull_points[fI].x;
                        m_field_hull_real_center.y += real_hull_points[fI].y;
                    }
                    m_field_hull_real_center.x /= m_field_hull_real.size();
                    m_field_hull_real_center.y /= m_field_hull_real.size();
                }

                if (m_field_hull_real.size() > 3) {
                    // 又把实际的坐标值用折线进行近似
                    cv::approxPolyDP(m_field_hull_real, m_field_hull_real, cv::arcLength(m_field_hull_real, true) * parameters.field.approxPoly, true);
                }

                // 终于画出了凸包轮廓 TAT
                if (parameters.field.showResult && parameters.monitor.update_gui_img) {
                    cv::drawContours(m_gui_img, hulls, -1, yellowColor(), 2, 8);
                }

                field_detection_OK = true;
            }
        }
    }

    if (parameters.field.enable && !field_detection_OK) {
        ROS_ERROR("Detecting field failed.");
    }

    /*****************
     * Line Detector *
     *****************/

    bool line_detection_OK = false;

    // calculate canny image
    cv::Mat channels[3], canny_img;
    cv::split(m_hsv_img, channels);
    m_canny_img_in_field = cv::Mat::zeros(m_canny_img_in_field.size(), CV_8UC1);
    // 对v进行模糊,模糊结果存在canny_img中
    cv::blur(channels[2], canny_img, cv::Size(parameters.line.blurSize, parameters.line.blurSize));
    cv::Canny(canny_img, canny_img, parameters.line.cannyThreadshold, parameters.line.cannyThreadshold * 3, parameters.line.cannyaperture);
    if (parameters.line.showCanny) {
        // fuck
    }
    canny_img.copyTo(m_canny_img_in_field, m_field_convex_hull);

    std::vector<LineSegment> clustered_lines;
    std::vector<LineSegment> clustered_lines_rotated;

    // detect white lines
    if (field_detection_OK && parameters.line.enable) {
        cv::Rect top_view_box;
        // top_view.width = field_model.field_length = 900
        top_view_box.x = -1 * parameters.field_model.field_length;
        top_view_box.y = -1 * parameters.field_model.field_length;
        top_view_box.width = 2 * parameters.field_model.field_length;
        top_view_box.height = 2 * parameters.field_model.field_length;
        std::vector<LineSegment> result_lines, result_lines_real;

        // get unmerged lines
        if (m_line.GetLines(m_hsv_img,
                            field_binary_raw,
                            m_gui_img,
                            // m_projection,
                            parameters.monitor.update_gui_img,
                            m_canny_img_in_field,
                            // cv::Rect(0, 0, parameters.camera.width, parameters.camera.height),
                            result_lines)) {
            // draw unmerged lines
            if (parameters.monitor.update_gui_img) {
                for (auto line : result_lines) {
                    cv::line(m_gui_img, line.P1, line.P2, blueMeloColor(), 3, 8);
                }
            }

            // merge lines
            if (m_projection.getOnRealCoordinate(result_lines, result_lines_real)) {
                if (MergeLinesMax(result_lines_real, parameters.line.AngleToMerge, parameters.line.DistanceToMerge, clustered_lines, top_view_box)) {
                    line_detection_OK = true;
                }
            }

            // draw merged lines
            if (parameters.line.showResult && parameters.monitor.update_gui_img) {
                std::vector<LineSegment> clustered_lines_img;
                if (m_projection.getOnImageCoordinate(clustered_lines, clustered_lines_img)) {
                    for (size_t i = 0; i < clustered_lines_img.size(); i++) {
                        line(m_gui_img, clustered_lines_img[i].P1, clustered_lines_img[i].P2, greenColor(), 3, 8);
                        circle(m_gui_img, clustered_lines_img[i].P1, 2, blueColor(), 2, 8);
                        circle(m_gui_img, clustered_lines_img[i].P2, 2, blueColor(), 2, 8);
                    }
                }
            }
        }
    }

    // if (parameters.line.enable && !field_detection_OK) {
    //     ROS_ERROR("Detecting lines failed.");
    // }

    /*******************
     * Circle detector *
     *******************/

    bool circle_detected = false;
    bool confused = false;
    cv::Point2d result_circle;
    cv::Point2d result_circle_rotated;

    if (field_detection_OK && parameters.circle.enable) {
        if (m_circle.GetCircle(parameters.field_model.center_circle_diameter / 2, clustered_lines, confused, result_circle)) {
            circle_detected = true;
        }
    }

    // if (parameters.circle.enable && !circle_detected) {
    //     ROS_ERROR("Detecting circle failed.");
    // }

    /*****************
     * Goal Detector *
     *****************/

    bool goal_detection_OK = false;

    std::vector<cv::Point2f> goal_position_real;
    std::vector<cv::Point2f> goal_position_real_rotated;

    if (field_detection_OK && parameters.goal.enable) {
        std::vector<LineSegment> result_lines, all_lines;
        // detect goal position
        goal_detection_OK =
          m_goal.GetPosts(canny_img, m_hsv_img, m_gray_img, m_goal_binary.clone(), m_projection, hull_field, result_lines, all_lines, goal_position_real, parameters.monitor.update_gui_img, m_gui_img);

        // draw all possible goal lines
        if (parameters.goal.showAllLines && parameters.monitor.update_gui_img) {
            for (auto line : all_lines) {
                cv::line(m_gui_img, line.P1, line.P2, blueMeloColor(), 2, 8);
            }
        }

        if (goal_detection_OK) {
            // draw selected goal lines
            if (parameters.goal.showResLine && parameters.monitor.update_gui_img) {
                for (auto line : result_lines) {
                    cv::line(m_gui_img, line.P1, line.P2, blueMeloColor(), 2, 8);
                }
            }
        }
    }

    // if (parameters.goal.enable && !goal_detection_OK) {
    //     ROS_ERROR("Detecting goal failed.");
    // }

    /****************
     * Localization *
     ****************/
    bool loc_detection_OK = false;

    // Rotate everthing!
    m_projection.CalcHeadingOffset(clustered_lines, circle_detected, result_circle, goal_position_real);
    m_field_hull_real_rotated = m_projection.RotateTowardHeading(m_field_hull_real);
    clustered_lines_rotated = m_projection.RotateTowardHeading(clustered_lines);
    goal_position_real_rotated = m_projection.RotateTowardHeading(goal_position_real);
    result_circle_rotated = m_projection.RotateTowardHeading(result_circle);

    if (parameters.loc.enable) {
        std::vector<LineContainer> all_lines;
        std::vector<FeatureContainer> all_features;
        if (m_loc.Calculate(clustered_lines_rotated, circle_detected, m_field_hull_real_center, m_field_hull_real, result_circle_rotated, goal_position_real_rotated, all_lines, all_features)) {
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
DVision::motionCallback(const dmotion::ActionCmd::ConstPtr& motion_msg) {
  m_action_cmd = *motion_msg;
  std::cout << "fuck: " << m_action_cmd.cmd_head.y << " " <<m_action_cmd.cmd_head.z << std::endl;
  m_pitch = static_cast<int>(m_action_cmd.cmd_head.y);
  m_yaw = static_cast<int>(m_action_cmd.cmd_head.z);
}

void
DVision::saveImgCallback(const SaveImg::ConstPtr& save_img_msg){
  m_save_img = *save_img_msg;
  if(m_save_img.IsSaveImg){
    auto frame = m_camera.capture();
    std::string path_str;
    path_str = "p_" + std::to_string(m_pitch) + "_y_" + std::to_string(m_yaw) + " ";
    frame.save(path_str);
  }
}

} // namespace dvision
