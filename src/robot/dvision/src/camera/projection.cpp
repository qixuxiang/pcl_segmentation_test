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

// Single point

bool
Projection::getOnImageCoordinate(const Point& point, Point2f& resPoint)
{
    return true;
}

bool
Projection::getOnRealCoordinate(const Point2f& point, Point& resPoint)
{
    return true;
}

// Points

bool
Projection::getOnImageCoordinate(const vector<Point>& points, vector<Point2f>& resPoints)
{
    if (!m_dist.undistort(points, resPoints)) {
        return false;
    }
    return true;
}

bool
Projection::getOnRealCoordinate(const vector<Point2f>& points, vector<Point>& resPoints)
{
    return true;
}

} // namespace dvision
