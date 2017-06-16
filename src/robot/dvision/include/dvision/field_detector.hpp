/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:45:41+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: field_detector.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:27+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/idetector.hpp"
#include "dvision/projection.hpp"
#include <ros/ros.h>
#include <vector>

namespace dvision {
class FieldDetector : public IDetector
{
  public:
    explicit FieldDetector();
    ~FieldDetector();
    bool Init();
    bool Process(cv::Mat& m_hsv_img, cv::Mat& m_gui_img, Projection& m_projection);

    bool GetPoints(std::vector<cv::Point>& res_points, std::vector<std::vector<cv::Point>>& all_field_contours);
    void FindInField(const cv::Mat& src_hsv_img, const cv::Mat& template_gray_img, cv::Mat* dst_gray_imgs, HSVRange* ranges, bool* in_template, int size = 1);
    std::vector<cv::Point> GetBodyMaskContourInRaw(float rot);

    inline std::vector<cv::Point>& hull_field()
    {
        return hull_field_;
    }

    inline cv::Mat& field_binary_raw()
    {
        return field_binary_raw_;
    }

    inline cv::Mat& field_convex_hull()
    {
        return field_convex_hull_;
    }

    inline cv::Point2f& field_hull_real_center()
    {
        return field_hull_real_center_;
    }

    inline std::vector<cv::Point2f>& field_hull_real()
    {
        return field_hull_real_;
    }

    inline std::vector<cv::Point2f>& field_hull_real_rotated()
    {
        return field_hull_real_rotated_;
    }

  private:
    std::vector<cv::Point> body_mask_contour_inverted_;
    std::vector<cv::Point> hull_field_;
    std::vector<cv::Point2f> field_hull_real_;
    std::vector<cv::Point2f> field_hull_real_rotated_;
    cv::Point2f field_hull_real_center_;
    cv::Mat field_binary_, field_binary_raw_;
    cv::Mat field_convex_hull_;
};
} // namespace dvision
