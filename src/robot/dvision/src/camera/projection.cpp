#include "dvision/projection.hpp"
#include "dvision/parameters.hpp"
#include <numeric>
using namespace std;
using namespace cv;

namespace dvision {
Projection::Projection()
{
}

void
Projection::init(ros::NodeHandle* nh)
{
    parameters.init(nh);
    m_dist.init();
}

Projection::~Projection()
{
}

bool
Projection::updateExtrinsic(double yaw, double pitch)
{
    m_yaw = yaw;
    m_pitch = pitch;
    // m_extrinsic = fuck;
    return true;
}

// Single point

bool
Projection::getOnImageCoordinate(const Point2f& point, Point& resPoint)
{
    ROS_ERROR("Not implemented");
    return true;
}

bool
Projection::getOnRealCoordinate(const Point& point, Point2f& resPoint)
{
    ROS_ERROR("Not implemented");
    return true;
}

// Points

bool
Projection::getOnImageCoordinate(const vector<Point2f>& points, vector<Point>& resPoints)
{
    // if (!m_dist.undistort(points, resPoints)) {
    //     return false;
    // }
    ROS_ERROR("Not implemented");
    return true;
}

bool
Projection::getOnRealCoordinate(const vector<Point>& points, vector<Point2f>& resPoints)
{
    ROS_ERROR("Not implemented");
    return true;
}

// Lines
bool
Projection::getOnImageCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines)
{
    ROS_ERROR("Not implemented");
    return true;
}

bool
Projection::getOnRealCoordinate(const std::vector<LineSegment>& lines, std::vector<LineSegment>& res_lines)
{
    ROS_ERROR("Not implemented");
    return true;
}

std::vector<cv::Point2f>
Projection::RotateTowardHeading(const std::vector<cv::Point2f>& in)
{
    std::vector<cv::Point2f> out(in.size());
    for (size_t i = 0; i < in.size(); i++) {
        out[i] = RotateTowardHeading(in[i]);
    }
    return out;
}

cv::Point2d
Projection::RotateTowardHeading(const cv::Point2d& in)
{
    return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

cv::Point2f
Projection::RotateTowardHeading(const cv::Point2f& in)
{
    return RotateAroundPoint(in, -Radian2Degree(GetHeading()));
}

std::vector<LineSegment>
Projection::RotateTowardHeading(const std::vector<LineSegment>& in)
{
    std::vector<LineSegment> out(in.size());
    for (size_t i = 0; i < in.size(); i++) {
        out[i] = LineSegment(RotateTowardHeading(in[i].P1), RotateTowardHeading(in[i].P2), in[i].GetProbability());
    }
    return out;
}

bool
Projection::CalcHeadingOffset(std::vector<LineSegment>& clustered_lines, bool circle_detected, const Point2d& result_circle, const std::vector<cv::Point2f>& goal_position)
{
    // Ver line in robot coord
    LineSegment VerLine(cv::Point(0, -10), cv::Point(0, 10));
    std::vector<double> heading_offset_vec;
    for (auto line_seg : clustered_lines) {
        if (line_seg.GetLength() > parameters.loc.minLineLen) {
            double angle_diff_ver = line_seg.GetExteriorAngleDegree(VerLine);
            if (angle_diff_ver < -90)
                angle_diff_ver += 180;
            if (angle_diff_ver > 90)
                angle_diff_ver += -180;

            if (circle_detected && DistanceFromLineSegment(line_seg, result_circle) < 50) {
                heading_offset_vec.push_back(angle_diff_ver);
                // cout << "CalcHeadingOffset: HorCenter angle " <<
                // heading_offset_vec.back()
                //      << endl;
            }
            if (goal_position.size() >= 2 && line_seg.DistanceFromLine(goal_position[0]) < 50 && line_seg.DistanceFromLine(goal_position[1]) < 50) {
                heading_offset_vec.push_back(angle_diff_ver);
                // cout << "CalcHeadingOffset: goal line angle " <<
                // heading_offset_vec.back()
                //      << endl;
            }
        }
    }
    if (!heading_offset_vec.empty()) {
        double heading_offset_avg = accumulate(heading_offset_vec.begin(), heading_offset_vec.end(), 0.0) / heading_offset_vec.size();
        // cout << "CalcHeadingOffset: " << heading_offset_avg << endl;
        m_heading_offset = M_PI / 180 * heading_offset_avg;
    }
}

} // namespace dvision
