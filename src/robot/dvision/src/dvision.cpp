// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle* n)
  : DProcess(VISION_FREQ, false)
  , m_nh(n)
{
    m_projection.init(n);
    m_concurrent.push([] {
        //     ROS_INFO("concurrent");
    });
}

DVision::~DVision()
{
}

void
DVision::tick()
{
    ROS_INFO("dvision tick");
    auto frame = m_camera.capture();

    double yaw = 0;
    double pitch = 0;
    m_projection.updateExtrinsic(yaw, pitch);
    m_projection.calcHomography();

    m_concurrent.spinOnce();
    m_concurrent.join();
}
} // namespace dvision
