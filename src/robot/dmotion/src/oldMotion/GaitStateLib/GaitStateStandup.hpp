#pragma once

#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"
#include <fstream>

class GaitStateStandup : public GaitStateBase {
 public:
  GaitStateStandup(I_HumanRobot* robot);
  ~GaitStateStandup();


  void execute() override;
  void entry() override;
  void exit() override;

};
