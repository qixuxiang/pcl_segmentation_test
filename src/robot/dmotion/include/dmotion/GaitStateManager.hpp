#pragma once

#include "dmotion/ActionCommand.h"
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "dmotion/MotionInfo.h"
#include "dmotion/GetDelta.h"
#include "std_msgs/String.h"
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
    void checkNewCommand(const dmotion::ActionCommand& request);

    void platCtrl(double& yaw, double& pitch);
    void reload_gaitdata(const std_msgs::String::ConstPtr&);
    void setCmd(dmotion::ActionCommand cmd);

    GaitStateBase* gaitState;
    GaitStateBase* goal_gaitState;
    GaitStateBase* prior_gaitState;
    dmotion::MotionInfo m_motion_info;

    bool getDelta(dmotion::GetDelta::Request& req, dmotion::GetDelta::Response& res);

  private:
    ros::NodeHandle* m_nh;
    ros::ServiceServer m_deltaServer;
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

    dmotion::ActionCommand m_cmd;
    double desYaw;
    double desPitch;
    ros::Time last_unstable_timestamp;
    ros::Time m_start_time;

    /* for head plat compensation */
    VecPos estimated_plat;
    geometry_msgs::Vector3 m_delta;

    bool m_simulation;
    int m_robotId;
    ros::Publisher m_pub;
    ros::Subscriber m_sub_reload_config;
};
