#pragma once

#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"

class GaitStateSetupLeftDown : public GaitStateBase {
 public:
  GaitStateSetupLeftDown(I_HumanRobot* robot, GaitStateManager* manager);
  ~GaitStateSetupLeftDown();


  void exit() override;
  void entry() override;
  void execute() override;

  GaitStateBase* setupback;

 private:
  void readOptions();
  double m_setupside_ankle_theta;
  double m_setupside_arm_theta;
  GaitStateManager* manager;
  int length;
  double* data[19];
};
