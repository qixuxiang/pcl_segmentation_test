/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-04T12:45:34+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: utils.cpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-04T12:46:40+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/utils.hpp"

namespace dvision {
// anything that avoids division overflow
#define SMALL_NUM 0.00000001
// dot product (3D) which allows std::vector operations in arguments
// distance = norm of difference
#define d(u, v) norm(u - v)
#define sign(a) (((a) < 0) ? -1 : ((a) > 0))

cv::Point2f
RotateCoordinateAxis(const double& alpha, const cv::Point2f& p)
{
    double alpha_rad = Degree2Radian(alpha);
    return cv::Point2f(static_cast<float>(p.x * std::cos(alpha_rad) - p.y * std::sin(alpha_rad)), static_cast<float>(p.x * std::sin(alpha_rad) + p.y * std::cos(alpha_rad)));
}

void
RotateCoordinateAxis(const double& alpha, const cv::Point2d& p, cv::Point2d& res)
{
    double alpha_rad = Degree2Radian(alpha);
    res = cv::Point2d(static_cast<float>(p.x * std::cos(alpha_rad) - p.y * std::sin(alpha_rad)), static_cast<float>(p.x * std::sin(alpha_rad) + p.y * std::cos(alpha_rad)));
}

cv::Point2f
RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha)
{
    cv::Point2f pCenter(0, 0);
    cv::Point2f p = pQuery - pCenter;
    return RotateCoordinateAxis(-alpha, p) + pCenter;
}

cv::Point2f
RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha, const cv::Point2f& pCenter)
{
    cv::Point2f p = pQuery - pCenter;
    return RotateCoordinateAxis(-alpha, p) + pCenter;
}

void
RotateAroundPoint(const cv::Point2d& pQuery, const double& alpha, cv::Point2d& res)
{
    cv::Point2d pCenter(0, 0);
    cv::Point2d p = pQuery - pCenter;
    cv::Point2d resRot;
    RotateCoordinateAxis(-alpha, p, resRot);
    res = resRot + pCenter;
}

double
Radian2Degree(const double& r)
{
    return (r / M_PI) * (180);
}

double
Degree2Radian(const double& d)
{
    return (d * M_PI) / (180);
}

double
GetDistance(const cv::Point2d& p)
{
    return std::sqrt((p.x * p.x) + (p.y * p.y));
}

float
GetDistance(const cv::Point2f& p)
{
    return std::sqrt((p.x * p.x) + (p.y * p.y));
}

double
GetDistance(const cv::Point2d& p, const cv::Point2d& p2)
{
    double x = std::abs(p.x - p2.x);
    double y = std::abs(p.y - p2.y);
    return std::sqrt(x * x + y * y);
}

int
Top(const cv::Rect& rec)
{
    return rec.x;
}

int
Bottom(const cv::Rect& rec)
{
    return rec.y;
}

int
Left(const cv::Rect& rec)
{
    return rec.x + rec.width;
}

int
Right(const cv::Rect& rec)
{
    return rec.y + rec.height;
}

cv::Scalar
grayWhite()
{
    return cv::Scalar(255);
}

cv::Scalar
pinkColor()
{
    return cv::Scalar(255, 0, 255);
}

cv::Scalar
pinkMeloColor()
{
    return cv::Scalar(183, 50, 130);
}

cv::Scalar
whiteColor()
{
    return cv::Scalar(255, 255, 255);
}

cv::Scalar
redColor()
{
    return cv::Scalar(0, 0, 255);
}

cv::Scalar
darkOrangeColor()
{
    return cv::Scalar(48, 163, 201);
}

cv::Scalar
redMeloColor()
{
    return cv::Scalar(51, 123, 255);
}

cv::Scalar
greenColor()
{
    return cv::Scalar(0, 255, 0);
}

cv::Scalar
yellowColor()
{
    return cv::Scalar(0, 255, 255);
}

cv::Scalar
blueColor()
{
    return cv::Scalar(255, 0, 0);
}

cv::Scalar
blueMeloColor()
{
    return cv::Scalar(255, 153, 51);
}

cv::Scalar
blackColor()
{
    return cv::Scalar(0, 0, 0);
}

cv::Scalar
blackGary()
{
    return cv::Scalar(0);
}

bool
MergeLinesMax(const std::vector<LineSegment>& resLinesReal, const double& maxDegree, const double& maxDistance, std::vector<LineSegment>& clusteredLines, const cv::Rect& box, const bool& useBounding)
{
    if (!MergeLinesOnce(resLinesReal, maxDegree, maxDistance, clusteredLines, box, useBounding)) {
        return false;
    }
    std::vector<LineSegment> before;
    before.reserve(resLinesReal.size());
    do {
        before = clusteredLines;
        if (!MergeLinesOnce(before, maxDegree, maxDistance, clusteredLines, box, useBounding)) {
            return false;
        }
    } while (before.size() > clusteredLines.size());
    return true;
}

bool
MergeLinesOnce(const std::vector<LineSegment>& resLinesReal, const double& maxDegree, const double& maxDistance, std::vector<LineSegment>& clusteredLines, const cv::Rect& box, const bool& useBounding)
{
}

float
dist3D_Segment_to_Segment(const LineSegment& S1, const LineSegment& S2)
{
}

// Normalize to [0,360):
double
CorrectAngleDegree360(const double& x)
{
}

// Normalize to [-180,180)
double
CorrectAngleDegree180(const double& x)
{
}

// Normalize to [0,360):
double
CorrectAngleRadian360(const double& x)
{
}

// Normalize to [-180,180)
double
CorrectAngleRadian180(const double& x)
{
}

// Normalize to [-180,180)
float
AngleDiffDegree180(const float& first, const float& second)
{
}

// Normalize to [-180,180)
float
AngleDiffRadian180(const float& first, const float& second)
{
}

cv::Point2f
GetAverage(const cv::Point2f& p0, const cv::Point2f& p1)
{
}

cv::Point2f
GetWeightedAverage(const cv::Point2f& p0, const cv::Point2f& p1, const float& w0, const float& w1)
{
}

float
GetWeightedAverage(const float& p0, const float& p1, const float& w0, const float& w1)
{
}

float
DistanceFromLineSegment(const LineSegment& line, const cv::Point2f& p)
{
}
} // namespace dvision
