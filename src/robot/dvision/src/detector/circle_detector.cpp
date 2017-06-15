/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T18:49:17+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: circle_detector.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:48:20+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/circle_detector.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
CircleDetector::CircleDetector()
{
}

CircleDetector::~CircleDetector()
{
}

bool
CircleDetector::Init()
{
    ROS_INFO("CircleDetector Init() finished");
    return true;
}

bool
CircleDetector::Process(cv::Point2d& result_circle, std::vector<LineSegment>& clustered_lines)
{
    if (parameters.circle.enable)
        return GetCircle(parameters.field_model.center_circle_diameter / 2, clustered_lines, result_circle);
    else
        return false;
}

bool
CircleDetector::GetCircle(const double& H2, std::vector<LineSegment>& clustered_lines, cv::Point2d& result_circle)
{
    // H2 1.5 / 2 = 0.75 貌似是圆的半径
    std::vector<cv::Point2d> circle_point;
    std::vector<LineSegment> clustered_lines_img;
    for (size_t line_i = 0; line_i < clustered_lines.size(); line_i++) {
        // 遍历clusteredLines，取出实际线的长度
        double l_len = clustered_lines[line_i].GetLength();
        // 如果线的长度超过了中心圆直线的长度的范围就continue掉了
        if (l_len > parameters.circle.maxLineLen || l_len < parameters.circle.minLineLen) {
            continue;
        }

        // 相对与原来的线垂直的线片段, 和原来直线等长的中垂线
        LineSegment pls = clustered_lines[line_i].PerpendicularLineSegment();

        // 再遍历剩下的直线
        for (size_t line_j = line_i + 1; line_j < clustered_lines.size(); line_j++) {
            // 获得长度并判断长度范围
            double l_len_2 = clustered_lines[line_j].GetLength();
            if (l_len_2 > parameters.circle.maxLineLen || l_len_2 < parameters.circle.minLineLen) {
                continue;
            }
            // 如果两条线段（注意是原来的线段，不是垂直的）最近的点距离 > 0.3 就跳过
            // dist3D_Segment_to_Segment : get the 3D minimum distance between 2
            // segments
            // parameters.circle->maxDistBetween2LS() = 0.3
            if (dist3D_Segment_to_Segment(clustered_lines[line_j], clustered_lines[line_i]) > parameters.circle.maxDistBetween2LS)
                continue;
            // 得到第二条线段的垂直的线段
            LineSegment pls2 = clustered_lines[line_j].PerpendicularLineSegment();
            cv::Point2d intersect;
            // 得到两条垂直点的交点
            if (pls.IntersectLineForm(pls2, intersect)) {
                // 求交点到初始线段的距离
                double distance1 = clustered_lines[line_i].DistanceFromLine(intersect);
                double distance2 = clustered_lines[line_j].DistanceFromLine(intersect);
                // 如果两个距离都符合要求， 就存到circlePoint的vector中
                if (distance1 < H2 * parameters.circle.radiusMaxCoef && distance1 > H2 * parameters.circle.radiusMinCoef && distance2 < H2 * parameters.circle.radiusMaxCoef &&
                    distance2 > H2 * parameters.circle.radiusMinCoef) {
                    // cout << "circle line length:" << l_len << ":" << l_len_2 <<
                    // endl;
                    circle_point.push_back(intersect);
                }
            }
        }
    }
    // cout << "circlePointSieze:" << circle_point.size() << endl;

    // 如果候选的点超过里三个，意味着至少有四条相邻的直线
    // 对所有点的坐标取均值，得到结果点，如果其中有点偏离比较大的，就设置confused为true
    if (circle_point.size() >= (size_t)parameters.circle.minLineSegmentCount) {
        cv::Point2d sum;
        for (size_t cCounter = 0; cCounter < circle_point.size(); cCounter++) {
            sum += circle_point[cCounter];
            // for (size_t cCounter2 = cCounter + 1; cCounter2 < circle_point.size(); cCounter2++) {
            //     if (GetDistance(circle_point[cCounter], circle_point[cCounter2]) > parameters.circle.confiusedDist) {
            //         confiused = true;
            //     }
            // }
        }
        result_circle.x = sum.x / circle_point.size();
        result_circle.y = sum.y / circle_point.size();
        // ROD_DEBUG("circleCentre: (%f, %f)", result_circle.x, result_circle.y);

        return true;
    }
    return false;
}

} // namespace dvision
