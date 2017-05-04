#pragma once

#include "dmotion/ActionCmd.h"
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "transitHub.hpp"

class GaitStateBase;
class GaitStateCrouch;
class GaitStateStandup;
class GaitStateKick;
class GaitStateGoalie;
class GaitStateSetupFrontDown;
class GaitStateSetupBackDown;
class GaitStateWenxi;

class GaitStateManager {
 public:
  GaitStateManager();
  ~GaitStateManager();
  void tick();
  void checkNewCommand(const dmotion::ActionCmd& request);

  void platCtrl(double& yaw, double& pitch);
  void reload_gaitdata();

  GaitStateBase* gaitState;
  GaitStateBase* goal_gaitState;
  GaitStateBase* prior_gaitState;

 private:
  /* init all gait states */
  void init_allstates();
  void init();
  I_HumanRobot* robot;
  RobotStatus* rstatus;
  transitHub* port;

  /* all gait state pointers , decouple*/
  GaitStateCrouch* crouch;
  // GaitStateStep* step;
  GaitStateStandup* standup;
  GaitStateKick* kick;
  GaitStateWenxi* walk;

  GaitStateGoalie* goalie;
  GaitStateSetupFrontDown* setupfrontdown;
  GaitStateSetupBackDown* setupbackdown;

  /* for head control, get head request in 100fps */
  double desYaw;
  double desPitch;
  int64_t last_unstable_timestamp;

  /* for head plat compensation */
  VecPos estimated_plat;

  deltadataDebug tempD;

  double head_k;
};
