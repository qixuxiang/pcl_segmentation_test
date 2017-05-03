#pragma once
#include <fstream>
#include "GaitStateBase.hpp"
#include "motion/GaitStateManager.hpp"

class GaitStateSetupFrontDown : public GaitStateBase {
 public:
  GaitStateSetupFrontDown(I_HumanRobot* robot, GaitStateManager* manager);

  ~GaitStateSetupFrontDown();
  void entry() override;
  void execute() override;
  void exit() override;
  GaitStateBase* setupback;

  void loadGaitFile() override; // for reload

 private:

  bool loadFile(const char* filename, double** t_data);

  bool loadData(double** t_data);

  std::fstream file;
  int length;
  double* data[10];
  GaitStateManager* manager;
};
