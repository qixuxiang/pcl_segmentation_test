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
    ROS_INFO("FieldDetector Init() started");
    return true;
}

FieldDetector::~FieldDetector()
{
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
