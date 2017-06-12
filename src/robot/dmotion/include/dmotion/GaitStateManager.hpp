#pragma once

#include "dmotion/ActionCmd.h"
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "transitHub.hpp"
#include <ros/ros.h>

class GaitStateBase;
class GaitStateCrouch;
class GaitStateStandup;
class GaitStateKick;
class GaitStateGoalie;
class GaitStateSetupFrontDown;
class GaitStateSetupBackDown;
class GaitStateWenxi;

class GaitStateManager
{
  public:
    explicit GaitStateManager(ros::NodeHandle* nh);
    ~GaitStateManager();
    void tick();
    void checkNewCommand(const dmotion::ActionCmd& request);

    void platCtrl(double& yaw, double& pitch);
    void reload_gaitdata();

    GaitStateBase* gaitState;
    GaitStateBase* goal_gaitState;
    GaitStateBase* prior_gaitState;

  private:
    ros::NodeHandle* m_nh;
    /* init all gait states */
    void init_allstates();
    void init();
    I_HumanRobot* robot;
    RobotStatus* rstatus;
    transitHub* port;

    /* all gait state pointers , decouple*/
    GaitStateCrouch* crouch;
    GaitStateStandup* standup;
    GaitStateKick* kick;
    GaitStateWenxi* walk;
    GaitStateGoalie* goalie;
    GaitStateSetupFrontDown* setupfrontdown;
    GaitStateSetupBackDown* setupbackdown;

    dmotion::ActionCmd m_cmd;
    double desYaw;
    double desPitch;
    ros::Time last_unstable_timestamp;

    /* for head plat compensation */
    VecPos estimated_plat;
    deltadataDebug tempD;
};
