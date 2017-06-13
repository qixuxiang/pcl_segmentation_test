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
    bool Process(std::vector<cv::Point2f>& m_field_hull_real,
                 std::vector<cv::Point2f>& m_field_hull_real_rotated,
                 std::vector<cv::Point>& hull_field,
                 cv::Mat& m_field_binary,
                 cv::Mat& field_binary_raw,
                 cv::Mat& m_field_convex_hull,
                 cv::Mat& m_hsv_img,
                 cv::Mat& m_gui_img,
                 cv::Point2f& m_field_hull_real_center,
                 Projection& m_projection);

    bool GetPoints(cv::Mat& binary_frame, std::vector<cv::Point>& res_points, std::vector<std::vector<cv::Point>>& all_field_contours);
    void FindInField(const cv::Mat& src_hsv_img, const cv::Mat& template_gray_img, cv::Mat* dst_gray_imgs, HSVRange* ranges, bool* in_template, int size = 1);
    std::vector<cv::Point> GetBodyMaskContourInRaw(float rot);

    std::vector<cv::Point> body_mask_contour_inverted_;

  private:
};
} // namespace dvision
