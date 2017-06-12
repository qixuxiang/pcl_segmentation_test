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
    m_sub_action_cmd = m_nh->subscribe("/humanoid/ActionCmd", 1, &DVision::motionCallback, this);
    m_sub_save_img = m_nh->subscribe("/humanoid/SaveImg", 1, &DVision::saveImgCallback, this);
}

DVision::~DVision()
{
}

void
DVision::tick()
{
    ROS_INFO("dvision tick");
    auto frame = m_camera.capture();
    double pitch = 0;
    double yaw = 0;
    m_projection.updateExtrinsic(yaw, pitch);
    m_projection.calcHomography();

    m_concurrent.spinOnce();
    m_concurrent.join();
}

void
DVision::motionCallback(const dmotion::ActionCmd::ConstPtr& motion_msg) {
  m_action_cmd = *motion_msg;
  std::cout << "fuck: " << m_action_cmd.cmd_head.y << " " <<m_action_cmd.cmd_head.z << std::endl;
  m_pitch = static_cast<int>(m_action_cmd.cmd_head.y);
  m_yaw = static_cast<int>(m_action_cmd.cmd_head.z);


}

void
DVision::saveImgCallback(const SaveImg::ConstPtr& save_img_msg){
  m_save_img = *save_img_msg;
  if(m_save_img.IsSaveImg){
    auto frame = m_camera.capture();
    std::string path_str;
    path_str = "p_" + std::to_string(m_pitch) + "_y_" + std::to_string(m_yaw) + " ";
    frame.save(path_str);
  }
}

} // namespace dvision
