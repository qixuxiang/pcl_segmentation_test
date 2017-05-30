// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>

#pragma once
#include "dprocess/dconcurrent.hpp"
#include "dprocess/dprocess.hpp"
#include "dvision/camera.hpp"
#include "dvision/projection.hpp"

namespace dvision {
class DVision : public dprocess::DProcess<DVision>
{
  public:
    explicit DVision(ros::NodeHandle* n);
    ~DVision();
    void tick() override;

  private:
    ros::NodeHandle* m_nh;
    Camera m_camera;
    Projection m_projection;
    dprocess::DConcurrent m_concurrent;
};
} // namespace dvision
