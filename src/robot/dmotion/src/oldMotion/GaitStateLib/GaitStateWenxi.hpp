#pragma once

#include "GaitStateBase.hpp"

class GaitStateWenxi : public GaitStateBase {
 public:
  GaitStateWenxi(I_HumanRobot* robot);
  ~GaitStateWenxi();
  void execute() override;

 private:
  void readOptions();
  double length_max_l;
  double length_max_r;
  double length_max_f;
  double length_max_b;
};
