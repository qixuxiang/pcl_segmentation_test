#include "dvision/projection.hpp"
using namespace std;
using namespace cv;

namespace dvision {
Projection::Projection(ros::NodeHandle* nh)
  : m_nh(nh)
{
    init();
    m_dist.init(m_imageSize, m_cameraMatrix, m_distCoeff);
}

Projection::~Projection()
{
}

bool
Projection::update()
{
    // update camera location and orientation
}

bool
Projection::update(double yaw, double pitch)
{
    m_yaw = yaw;
    m_pitch = pitch;
}

bool
Projection::calcHomography()
{
    m_ipm.initGetHomography(m_cameraLocation, m_cameraOrientation, homoImgToReal, homoReaToImg);
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

#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!m_nh->getParam(x, y)) {                                                                                                                                                                   \
            ROS_FATAL("Projection get pararm error!");                                                                                                                                                 \
            throw std::runtime_error("Projection get param error!");                                                                                                                                   \
        }                                                                                                                                                                                              \
    } while (0)

void
Projection::init()
{
    double fx;
    double fy;
    double cx;
    double cy;
    int width;
    int height;
    vector<double> dist_coeff;

    GPARAM("/dvision/projection/fx", fx);
    GPARAM("/dvision/projection/fy", fy);
    GPARAM("/dvision/projection/cx", cx);
    GPARAM("/dvision/projection/cy", cy);
    GPARAM("/dvision/projection/dist_coeff", dist_coeff);
    GPARAM("/dvision/camera/width", width);
    GPARAM("/dvision/camera/height", height);

    m_cameraMatrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    m_distCoeff = Mat_<double>(1, 14);

    for (uint32_t i = 0; i < dist_coeff.size(); ++i) {
        m_distCoeff.at<double>(0, i) = dist_coeff[i];
    }

    m_imageSize = Size(width, height);
}

#undef GPARAM
} // namespace dvision
