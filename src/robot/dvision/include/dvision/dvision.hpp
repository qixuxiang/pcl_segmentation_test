// Created on: May 20, 2017
//     Author: Wenxing Mei <mwx36mwx@gmail.com>
#pragma once
#include "dprocess/dprocess.hpp"
using dprocess::DProcess;
namespace dvision {
class DVision : public DProcess<DVision> {
public:
  explicit DVision(ros::NodeHandle* n);
  ~DVision();
  void tick() override;

private:
  ros::NodeHandle* m_nh;
};
}