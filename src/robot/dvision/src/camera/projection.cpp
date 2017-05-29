#include "dvision/projection.hpp"

namespace dvision {
void Projection::init()
{
    //
}

void Projection::update()
{
    // update camera location and orientation
    // calculate homography
}

void
Projection::getOnImageCoordinate(const cv::Point& point, cv::Point2f& resPoint)
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
