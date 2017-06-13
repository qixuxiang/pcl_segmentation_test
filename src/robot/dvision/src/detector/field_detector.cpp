/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:46:50+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: field_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:14+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/field_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
FieldDetector::FieldDetector()
{
}

FieldDetector::~FieldDetector()
{
}

bool
FieldDetector::Init()
{
    // TODO(corenel) init body_mask_contour_inverted_
    //      cv::FileStorage fr(parameters.configPath+"BodyMask.yml",
    //                         cv::FileStorage::READ);
    //      cv::FileNode fileNode = fr["IGUS"];
    //      read(fileNode, body_mask_contour_inverted_);
    //      fr.release();
    //      if (body_mask_contour_inverted_.size() < 6)
    //      {
    //        ROS_ERROR("Create or modify BodyMask.yml!");
    //      }
    ROS_INFO("FieldDetector Init() finished");
    return true;
}

bool
FieldDetector::Process(std::vector<cv::Point2f>& m_field_hull_real,
                       std::vector<cv::Point2f>& m_field_hull_real_rotated,
                       std::vector<cv::Point>& hull_field,
                       cv::Mat& m_field_binary,
                       cv::Mat& field_binary_raw,
                       cv::Mat& m_field_convex_hull,
                       cv::Mat& m_hsv_img,
                       cv::Mat& m_gui_img,
                       cv::Point2f& m_field_hull_real_center,
                       Projection& m_projection)
{
    // clear field convex hull
    m_field_hull_real.clear();
    m_field_hull_real_rotated.clear();

    // create binary mask of field
    m_field_binary = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
    field_binary_raw = cv::Mat::zeros(m_hsv_img.size(), CV_8UC1);
    cv::inRange(m_hsv_img, cv::Scalar(parameters.field.h0, parameters.field.s0, parameters.field.v0), cv::Scalar(parameters.field.h1, parameters.field.s1, parameters.field.v1), field_binary_raw);
    m_field_binary = field_binary_raw.clone();

    // draw field mask on gui image
    if (parameters.field.showMask && parameters.monitor.update_gui_img) {
        cv::Mat darker = cv::Mat::zeros(m_gui_img.size(), CV_8UC3);
        darker.copyTo(m_gui_img, 255 - field_binary_raw);
    }

    // detect field
    std::vector<cv::Point> field_points;
    std::vector<cv::Point> field_contour_undist;
    std::vector<cv::Point> hull_undist;
    std::vector<cv::Point> hull_undist_mid_point;
    std::vector<std::vector<cv::Point>> all_field_contours;

    if (parameters.field.enable && GetPoints(m_field_binary, field_points, all_field_contours)) {
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
                std::vector<cv::Point> tmp_body_mask_contour = GetBodyMaskContourInRaw(-Radian2Degree(0));
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

                return true;
            }
        }
    }

    return false;
}

bool
FieldDetector::GetPoints(cv::Mat& binary_frame, std::vector<cv::Point>& res_points, std::vector<std::vector<cv::Point>>& all_field_contours)
{
    if (parameters.field.erode > 0) {
        erode(binary_frame, binary_frame, cv::Mat(), cv::Point(-1, -1), parameters.field.erode);
    }
    if (parameters.field.dilate > 0) {
        dilate(binary_frame, binary_frame, cv::Mat(), cv::Point(-1, -1), parameters.field.dilate);
    }
    if (parameters.field.erode2 > 0) {
        erode(binary_frame, binary_frame, cv::Mat(), cv::Point(-1, -1), parameters.field.erode2);
    }
    if (parameters.field.dilate2 > 0) {
        dilate(binary_frame, binary_frame, cv::Mat(), cv::Point(-1, -1), parameters.field.dilate2);
    }
    cv::findContours(binary_frame.clone(), all_field_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    res_points.reserve(parameters.field.maxContourCount);
    std::sort(all_field_contours.begin(), all_field_contours.end(), SortFuncDescending);
    bool ret = false;
    int total_result = 0;
    for (size_t i = 0; i < all_field_contours.size(); i++) {
        if (total_result >= parameters.field.maxContourCount) {
            return ret;
        }
        cv::Rect rec = cv::boundingRect(all_field_contours[i]);
        double area = cv::contourArea(all_field_contours[i]);
        if (std::abs(parameters.camera.height - Bottom(rec)) <= parameters.field.maxDownDiffPixel && area >= parameters.field.minArea) {
            cv::approxPolyDP(all_field_contours[i], all_field_contours[i], cv::arcLength(all_field_contours[i], true) * 0.003, true);
            for (size_t pIt = 0; pIt < all_field_contours[i].size(); pIt++) {
                res_points.push_back(all_field_contours[i][pIt]);
            }
            ret = true;
        }
        total_result++;
    }
    return ret;
}

void
FieldDetector::FindInField(const cv::Mat& src_hsv_img, const cv::Mat& template_gray_img, cv::Mat* dst_gray_imgs, HSVRange* ranges, bool* in_template, int size)
{
    const int src_size = src_hsv_img.rows * src_hsv_img.cols;
    int* indexs = new int[4];
    for (int k = 0; k < size; k++) {
        indexs[k] = 0;
    }
    uchar* src_hsv_img_D = src_hsv_img.data;
    uchar* template_gray_img_D = template_gray_img.data;

    for (int i = 0; i < src_size; i++) {
        ushort h = src_hsv_img_D[0], s = src_hsv_img_D[1], v = src_hsv_img_D[2];
        if (template_gray_img_D[0] >= 254) {
            for (int k = 0; k < size; k++) {
                if (h >= ranges[k].h0 && h <= ranges[k].h1 && s >= ranges[k].s0 && s <= ranges[k].s1 && v >= ranges[k].v0 && v <= ranges[k].v1) {
                    dst_gray_imgs[k].data[indexs[k]] = 255;
                } else {
                    dst_gray_imgs[k].data[indexs[k]] = 0;
                }
            }
        } else {
            for (int k = 0; k < size; k++) {
                if (in_template[k])
                    continue;
                if (h >= ranges[k].h0 && h <= ranges[k].h1 && s >= ranges[k].s0 && s <= ranges[k].s1 && v >= ranges[k].v0 && v <= ranges[k].v1) {
                    dst_gray_imgs[k].data[indexs[k]] = 255;
                } else {
                    dst_gray_imgs[k].data[indexs[k]] = 0;
                }
            }
        }
        template_gray_img_D += 1;
        src_hsv_img_D += 3;
        for (int k = 0; k < size; k++) {
            indexs[k]++;
        }
    }
    delete[] indexs;
}

std::vector<cv::Point>
FieldDetector::GetBodyMaskContourInRaw(float rot)
{
    std::vector<cv::Point> res;
    if (body_mask_contour_inverted_.size() < 6)
        return res;
    for (size_t i = 0; i < body_mask_contour_inverted_.size(); i++) {
        cv::Point rotated = RotateAroundPoint(body_mask_contour_inverted_[i], rot);
        cv::Point p(static_cast<int>(rotated.x + parameters.camera.width / 2.), abs(rotated.y - 480));
        if (p.inside(cv::Rect(0, 0, parameters.camera.width, parameters.camera.height))) {
            if (res.size() == 0) {
                res.push_back(cv::Point(p.x, parameters.camera.height));
            }
            res.push_back(p);
        }
    }
    if (res.size() > 0) {
        int xlast = res[res.size() - 1].x;
        res.push_back(cv::Point(xlast, parameters.camera.height));
    }
    return res;
}

} // namespace dvision
