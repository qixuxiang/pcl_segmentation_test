// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dvision/camera.hpp"
#include "dvision/projection.hpp"
#include "dvision/SaveImg.h"
#include "dmotion/ActionCmd.h"

namespace dvision {
class DVision : public dprocess::DProcess<DVision>
{
  public:
    explicit DVision(ros::NodeHandle* n);
    ~DVision();
    void tick() override;

  private:
    ros::NodeHandle* m_nh;
    ros::Subscriber m_sub_action_cmd;
    ros::Subscriber m_sub_save_img;
    Camera m_camera;
    Projection m_projection;
    dprocess::DConcurrent m_concurrent;
    int m_yaw;
    int m_pitch;
    dmotion::ActionCmd m_action_cmd;
    SaveImg m_save_img;
    void motionCallback(const dmotion::ActionCmd::ConstPtr& msg);
    void saveImgCallback(const SaveImg::ConstPtr& save_img_msg);
};
} // namespace dvision
