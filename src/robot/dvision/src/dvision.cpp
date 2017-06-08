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
    m_circle.Init();
    m_field.Init();
    m_goal.Init();
    m_line.Init();
    m_loc.Init();

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

    // TODO(mwx) get yaw and pitch from motor
    double yaw = 0;
    double pitch = 0;

    if (!m_projection.updateExtrinsic(yaw, pitch)) {
        ROS_ERROR("Cannot update extrinsic of camera!");
    }

    if (!m_projection.calcHomography()) {
        ROS_ERROR("Cannot calculate homography!");
    }
    if (!m_loc.Update(m_projection)) {
        ROS_ERROR("Cannot update localization!");
    }

    m_field_hull_real.clear();
    m_field_hull_real_rotated.clear();

    m_concurrent.spinOnce();
    m_concurrent.join();
}
} // namespace dvision
