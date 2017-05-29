#pragma once
#include "dmotion/GaitStateLib/GaitStateBase.hpp"
#include <fstream>

class GaitStateManager;
class GaitStateSetupBackDown : public GaitStateBase
{
  public:
    GaitStateSetupBackDown(I_HumanRobot* robot, GaitStateManager* manager);
    ~GaitStateSetupBackDown();
    void entry() override;
    void execute() override;
    void exit() override;

  private:
    int length;
    double* data[10];
    GaitStateManager* manager;
};
