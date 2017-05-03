#pragma once
#include <fstream>
#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"
class GaitStateSetupBackDown : public GaitStateBase {
 public:
  GaitStateSetupBackDown(I_HumanRobot* robot, GaitStateManager* manager);
  ~GaitStateSetupBackDown();
  void entry() override;
  void execute() override;
  void exit() override;
  void loadGaitFile() override;
 private:

  bool loadFile(const char* filename, double** t_data);

  bool loadData(double** t_data);

  fstream file;
  int length;
  double* data[10];
  GaitStateManager* manager;
};
