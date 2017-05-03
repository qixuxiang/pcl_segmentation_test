#pragma once

#include "GaitStateSupportLib/HumanRobotInterface.hpp"
#include "transitHub.hpp"
#include "ActionCommand.hpp"

/*******************************************************
 * pre definition of gaitstates
 *******************************************************/
class GaitStateBase;
class GaitStateCrouch;
class GaitStateStandup;
class GaitStateKick;
class GaitStateGoalie;
//class GaitStateGoalieright;
//class GaitStateGoaliemid;
class GaitStateSetupFrontDown;
class GaitStateSetupBackDown;
class GaitStateWenxi;

class GaitStateManager {
 public:
  GaitStateManager();
  ~GaitStateManager();
  void tick();
  void checkNewCommand(ActionCommand::All& request);

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

  GaitStateGoalieleft* goalieleft;
  GaitStateGoalieright* goalieright;
  GaitStateGoaliemid* goaliemid;
  GaitStateSetupFrontDown* setupfrontdown;
  GaitStateSetupBackDown* setupbackdown;
  GaitStateSetupLeftDown* setupleftdown;
  GaitStateSetupRightDown* setuprightdown;
  GaitStateWalkLeftkick* walkleftkick;
  GaitStateWalkRightkick* walkrightkick;

  /* for head control, get head request in 100fps */
  double desYaw;
  double desPitch;
  int64_t last_unstable_timestamp;

  /* for head plat compensation */
  VecPos estimated_plat;

  deltadataDebug tempD;

  double head_k;
};
