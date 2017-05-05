#pragma once
#include <fstream>
#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateManager;
class GaitStateSetupBackDown : public GaitStateBase {
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
