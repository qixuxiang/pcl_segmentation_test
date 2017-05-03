#pragma once

#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"

class GaitStateSetupRightDown : public GaitStateBase {
 public:
  GaitStateSetupRightDown(I_HumanRobot* robot, GaitStateManager* manager);

  ~GaitStateSetupRightDown();


  void execute() override;
  void entry() override;
  void exit() override;
  GaitStateBase* setupback;

 private:
  void readOptions();
  double m_setupside_ankle_theta;
  double m_setupside_arm_theta;
  GaitStateManager* manager;
  int length;
  double* data[19];
};
