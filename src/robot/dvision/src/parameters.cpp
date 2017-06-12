#include "dvision/parameters.hpp"
#include <vector>
using std::vector;
using namespace cv;

namespace dvision {

#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!nh->getParam(x, y)) {                                                                                                                                                                   \
            ROS_FATAL("Projection get pararm error!");                                                                                                                                                 \
            throw std::runtime_error("Projection get param error!");                                                                                                                                   \
        }                                                                                                                                                                                              \
    } while (0)

void Parameters::init(ros::NodeHandle *nh) {
    vector<double> dist_coeff;

    GPARAM("/dvision/projection/fx", parameters.camera.fx);
    GPARAM("/dvision/projection/fy", parameters.camera.fy);
    GPARAM("/dvision/projection/cx", parameters.camera.cx);
    GPARAM("/dvision/projection/cy", parameters.camera.cy);
    GPARAM("/dvision/projection/dist_coeff", dist_coeff);
    GPARAM("/dvision/camera/width", parameters.camera.width);
    GPARAM("/dvision/camera/height", parameters.camera.height);

    parameters.camera.cameraMatrix = (Mat_<double>(3, 3) << parameters.camera.fx, 0, parameters.camera.cx, 0, parameters.camera.fy, parameters.camera.cy, 0, 0, 1);
    parameters.camera.distCoeff = Mat_<double>(1, 14);

    for (uint32_t i = 0; i < dist_coeff.size(); ++i) {
        parameters.camera.distCoeff.at<double>(0, i) = dist_coeff[i];
    }

    parameters.camera.imageSize = Size(parameters.camera.width, parameters.camera.height);
}

#undef GPARAM

Parameters parameters;
} // namespace dvision