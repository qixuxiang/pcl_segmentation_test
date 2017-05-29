#include "dvision/projection.hpp"

namespace dvision {
Projection::Projection(ros::NodeHandle* nh) : m_nh(nh)
{
    // TODO(MWX): read configurations
    ROS_ERROR("Unimplemented");
    m_dist.init(m_imageSize, m_cameraMatrix, m_distCoeff);
}

Projection::~Projection()
{
}

void
Projection::update()
{
    // update camera location and orientation
    // calculate homography
}

void
Projection::calcHomography()
{
}

void
Projection::getOnImageCoordinate(const std::vector<cv::Point>& points, std::vector<cv::Point2f>& resPoints)
{
}

void
Projection::getOnRealCoordinate(const cv::Point2f& point, cv::Point& resPoint)
{
}

void
Projection::getOnRealCoordinate(const std::vector<cv::Point2f>& points, std::vector<cv::Point>& resPoints)
{
}
} // namespace dvision
