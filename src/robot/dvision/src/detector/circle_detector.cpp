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

bool
CircleDetector::Init()
{
    ROS_INFO("CircleDetector Init() started");
    return true;
}

CircleDetector::~CircleDetector()
{
}

bool
CircleDetector::GetCircle(const double& H2, std::vector<LineSegment>& clusteredLines, bool& confiused, cv::Point2d& resultCircle)
{
    // H2 1.5 / 2 = 0.75 貌似是圆的半径
    std::vector<cv::Point2d> circlePoint;
    std::vector<LineSegment> clusteredLinesImg;
    for (size_t lineI = 0; lineI < clusteredLines.size(); lineI++) {
        // 遍历clusteredLines，取出实际线的长度
        double lLength = clusteredLines[lineI].GetLength();
        // 如果线的长度超过了中心圆直线的长度的范围就continue掉了
        if (lLength > parameters.circle.maxLineLen || lLength < parameters.circle.minLineLen) {
            continue;
        }

        // 相对与原来的线垂直的线片段, 和原来直线等长的中垂线
        LineSegment pls = clusteredLines[lineI].PerpendicularLineSegment();

        // 再遍历剩下的直线
        for (size_t lineJ = lineI + 1; lineJ < clusteredLines.size(); lineJ++) {
            // 获得长度并判断长度范围
            double lLength2 = clusteredLines[lineJ].GetLength();
            if (lLength2 > parameters.circle.maxLineLen || lLength2 < parameters.circle.minLineLen) {
                continue;
            }
            // 如果两条线段（注意是原来的线段，不是垂直的）最近的点距离 > 0.3 就跳过
            // dist3D_Segment_to_Segment : get the 3D minimum distance between 2
            // segments
            // parameters.circle->maxDistBetween2LS() = 0.3
            if (dist3D_Segment_to_Segment(clusteredLines[lineJ], clusteredLines[lineI]) > parameters.circle.maxDistBetween2LS)
                continue;
            // 得到第二条线段的垂直的线段
            LineSegment pls2 = clusteredLines[lineJ].PerpendicularLineSegment();
            cv::Point2d intersect;
            // 得到两条垂直点的交点
            if (pls.IntersectLineForm(pls2, intersect)) {
                // 求交点到初始线段的距离
                double distance1 = clusteredLines[lineI].DistanceFromLine(intersect);
                double distance2 = clusteredLines[lineJ].DistanceFromLine(intersect);
                // 如果两个距离都符合要求， 就存到circlePoint的vector中
                if (distance1 < H2 * parameters.circle.radiusMaxCoef && distance1 > H2 * parameters.circle.radiusMinCoef && distance2 < H2 * parameters.circle.radiusMaxCoef &&
                    distance2 > H2 * parameters.circle.radiusMinCoef) {
                    // cout << "circle line length:" << lLength << ":" << lLength2 <<
                    // endl;
                    circlePoint.push_back(intersect);
                }
            }
        }
    }
    // cout << "circlePointSieze:" << circlePoint.size() << endl;

    // 如果候选的点超过里三个，意味着至少有四条相邻的直线
    // 对所有点的坐标取均值，得到结果点，如果其中有点偏离比较大的，就设置confused为true
    if (circlePoint.size() >= (size_t)parameters.circle.minLineSegmentCount) {
        cv::Point2d sum;
        for (size_t cCounter = 0; cCounter < circlePoint.size(); cCounter++) {
            sum += circlePoint[cCounter];
            for (size_t cCounter2 = cCounter + 1; cCounter2 < circlePoint.size(); cCounter2++) {
                if (GetDistance(circlePoint[cCounter], circlePoint[cCounter2]) > parameters.circle.confiusedDist) {
                    confiused = true;
                }
            }
        }
        resultCircle.x = sum.x / circlePoint.size();
        resultCircle.y = sum.y / circlePoint.size();
        // ROD_DEBUG("circleCentre: (%f, %f)", resultCircle.x, resultCircle.y);

        return true;
    }
    return false;
}

} // namespace dvision
