#include "dvision/projection.hpp"
#include "dvision/parameters.hpp"
using namespace std;
using namespace cv;

namespace dvision {
Projection::Projection()
{
}

void Projection::init(ros::NodeHandle* nh)
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

void
Projection::getOnImageCoordinate(const Point& point, Point2f& resPoint)
{
}

void
Projection::getOnRealCoordinate(const Point2f& point, Point& resPoint)
{
}

void
Projection::getOnImageCoordinate(const vector<Point>& points, vector<Point2f>& resPoints)
{
}

void
Projection::getOnRealCoordinate(const vector<Point2f>& points, vector<Point>& resPoints)
{
}

} // namespace dvision
