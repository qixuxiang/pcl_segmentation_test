#pragma once

#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateGoalie : public GaitStateBase {
 public:
  GaitStateGoalie(I_HumanRobot* robot);
  ~GaitStateGoalie();
  void execute() override;
  void entry() override;
  void exit() override;

 private:
  void doLiftbothHand();
  void doGoalieMid();
  void doRecover();

  int m_stepnum;
  bool m_goalie_bool;
  int m_sleeptime;
  int m_zf_15;
  int m_zf_17;
};
