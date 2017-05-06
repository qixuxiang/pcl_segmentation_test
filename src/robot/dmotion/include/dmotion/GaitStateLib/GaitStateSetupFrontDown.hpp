#pragma once
#include <fstream>
#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateManager;
class GaitStateSetupFrontDown : public GaitStateBase {
 public:
  GaitStateSetupFrontDown(I_HumanRobot* robot, GaitStateManager* manager);
  ~GaitStateSetupFrontDown();
  void entry() override;
  void execute() override;
  void exit() override;
 private:
  int length;
  double* data[10];
  GaitStateManager* manager;
};
