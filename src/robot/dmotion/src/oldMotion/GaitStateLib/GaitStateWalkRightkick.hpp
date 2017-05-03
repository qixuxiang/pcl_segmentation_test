#pragma once

#include "GaitStateBase.hpp"

class GaitStateWalkRightkick : public GaitStateBase {
 public:
  GaitStateWalkRightkick(I_HumanRobot* robot);
  ~GaitStateWalkRightkick();

  void execute() override;
  void entry() override;
  // void exit() override;

 private:
  void readOptions();
  double m_AnkleH_mid_r;
  double m_kickexcute_r;
};
