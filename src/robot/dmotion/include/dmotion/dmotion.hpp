#pragma once
#include <dprocess/dprocess.hpp>
#include "dmotion/GaitStateManager.hpp"
#include "dmotion/ActionCmd.h"
using dprocess::DProcess;
namespace dmotion {
class DMotion: public DProcess<DMotion> {
public:
  explicit DMotion(ros::NodeHandle* n);
  ~DMotion();
  void tick() override;

private:
  ros::NodeHandle* m_nh;
  GaitStateManager m_manager;
  ActionCmd m_cmd;
  ros::Subscriber m_sub;
  void callback(const ActionCmd::ConstPtr& msg);
};
}