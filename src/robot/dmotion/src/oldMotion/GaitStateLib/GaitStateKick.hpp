#pragma once

#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"

class GaitStateKick : public GaitStateBase {
 public:
  GaitStateKick(I_HumanRobot* robot, GaitStateManager* manager);
  ~GaitStateKick();


  void exit() override;
  void entry() override;
  void leftKickEntry();
  void rightKickEntry();
  void execute() override;

  void loadGaitFile() override;


 private:

  GaitStateManager* manager;
  int lengthR_;
  int lengthL_;
  int length_;
  bool crouchBool_;
  double* dataR_[16];
  double* dataL_[16];
};
