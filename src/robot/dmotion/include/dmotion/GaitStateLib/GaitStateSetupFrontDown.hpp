#pragma once
#include <fstream>
#include "dmotion/GaitStateLib/GaitStateBase.hpp"

class GaitStateManager;
class GaitStateSetupFrontDown : public GaitStateBase {
 public:
  GaitStateSetupFrontDown(I_HumanRobot* robot, GaitStateManager* manager);
  ~GaitStateSetupFrontDown();
  void entry() override;
  void execute() override;
  void exit() override;
  GaitStateBase* setupback;
  void loadGaitFile() override;
 private:

  bool loadFile(const char* filename, double** t_data);
  bool loadData(double** t_data);

  std::fstream file;
  int length;
  double* data[10];
  GaitStateManager* manager;
};
