// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#include "dvision/dvision.hpp"

namespace dvision {
static const int VISION_FREQ = 30;
DVision::DVision(ros::NodeHandle* n)
  : DProcess(VISION_FREQ, false)
  , m_nh(n)
{
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
    // frame.show();
    //  frame.save("x.png");
    m_concurrent.spinOnce();
    m_concurrent.join();
}
} // namespace dvision
