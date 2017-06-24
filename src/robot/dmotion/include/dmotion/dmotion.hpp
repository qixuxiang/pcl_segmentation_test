#pragma once
#include "dmotion/ActionCommand.h"
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
    void setCmd(ActionCommand cmd);

  private:
    void prepareShutdown() override;
    //ros::NodeHandle* m_nh;
    GaitStateManager m_manager;
    ActionCommand m_cmd;
};
}
