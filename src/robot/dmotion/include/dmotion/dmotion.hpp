#pragma once
#include "dmotion/ActionCmd.h"
#include "dmotion/GaitStateManager.hpp"
#include <dprocess/dprocess.hpp>
using dprocess::DProcess;
namespace dmotion {
class DMotion : public DProcess<DMotion>
{
  public:
    explicit DMotion(ros::NodeHandle* n);
    virtual ~DMotion();
    void tick() override;

  private:
    void prepareShutdown() override;
    ros::NodeHandle* m_nh;
    GaitStateManager m_manager;
    ActionCmd m_cmd;
    ros::Subscriber m_sub;
    void callback(const ActionCmd::ConstPtr& msg);
};
}
