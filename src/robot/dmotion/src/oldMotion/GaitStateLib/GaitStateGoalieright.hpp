#pragma once

#include "GaitStateBase.hpp"

class GaitStateGoalieright : public GaitStateBase {
 public:
  GaitStateGoalieright(I_HumanRobot* robot);
  ~GaitStateGoalieright();

  void execute() override;

 private:
  void doLiftrightHand();
  void doGoalieRight();
  void doRecover();


};
