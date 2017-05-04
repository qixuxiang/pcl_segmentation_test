#pragma once
#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateCrouch : public GaitStateBase {
 public:
  GaitStateCrouch(I_HumanRobot*);
  ~GaitStateCrouch();
  void execute() override;
  void entry() override;
  void exit() override;
};
