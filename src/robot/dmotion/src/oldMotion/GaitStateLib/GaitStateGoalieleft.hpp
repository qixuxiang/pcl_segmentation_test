#pragma once

#include "GaitStateBase.hpp"

class GaitStateGoalieleft : public GaitStateBase {
 public:
  GaitStateGoalieleft(I_HumanRobot* robot);
  ~GaitStateGoalieleft();


  void execute() override;
  void entry() override;
  void exit() override;

 private:
  void doLiftleftHand();
  void doGoalieLeft();
  void doRecover();

  double m_goalie_theta;
  int m_stepnum;
  bool m_goalie_bool;
  double m_setupside_ankle_theta;
  int m_sleeptime;
};
