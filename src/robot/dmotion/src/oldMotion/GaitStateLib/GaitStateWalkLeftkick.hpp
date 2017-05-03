#pragma once

#include "GaitStateBase.hpp"

class GaitStateWalkLeftkick : public GaitStateBase {
 public:
  GaitStateWalkLeftkick(I_HumanRobot* robot);
  ~GaitStateWalkLeftkick();


  void execute() override;
  void entry() override;

 private:
  void readOptions();
  double m_AnkleH_mid_l;
  double m_kickexcute_l;
};
