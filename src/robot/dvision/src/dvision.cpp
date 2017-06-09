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
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_loc.Init();

    m_concurrent.push([] {
        //     ROS_INFO("concurrent");
    });
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
    double yaw = 0;
    double pitch = 0;

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

    // detect field
    if (parameters.field.enable && m_field.GetPoints(m_field_binary, field_points, all_field_contours)) {
        // show all field contours for debugging
        if (parameters.field.showDebug && parameters.monitor.update_gui_img) {
            cv::drawContours(m_gui_img, all_field_contours, -1, cv::Scalar(70, 220, 70), 2, 8);
        }

        // 计算非畸变下的轮廓点
        if (m_projection.undistort(field_points, field_contour_undist)) {
            std::vector<cv::Point> hull_undist, hull_undist_mid_point, hull_field;
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
            }
        }
    }

    /****************
     * Post process *
     ****************/

    m_concurrent.spinOnce();
    m_concurrent.join();
}
} // namespace dvision
