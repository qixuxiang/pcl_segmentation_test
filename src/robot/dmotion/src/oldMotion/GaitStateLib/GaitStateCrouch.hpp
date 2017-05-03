#pragma once

#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"
#include <fstream>

class GaitStateCrouch : public GaitStateBase {
 public:
  GaitStateCrouch(I_HumanRobot*);

  ~GaitStateCrouch();


  void execute() override;
  void entry() override;
  void exit() override;

};
