#include "dmotion/GaitStateManager.hpp"
#include "dmotion/GaitStateLib/AllGaitState.hpp"
#include "dmotion/GaitStateSupportLib/HumanRobot.hpp"
#include "dtransmit/dtransmit.hpp"
#include "dconfig/dconstant.hpp"
#include "dmotion/MotionData.hpp"

using namespace dmotion;
dtransmit::DTransmit* transmit;

GaitStateManager::GaitStateManager(ros::NodeHandle* nh)
  : m_nh(nh)
{
    if(!nh->getParam("/ZJUDancer/Simulation", m_simulation)) {
        ROS_ERROR("Get simulation param error");
    }
    if(!nh->getParam("/ZJUDancer/RobotId", m_robotId)) {
        ROS_ERROR("Get robotId param error");
    }

    init();
    init_allstates();
    gaitState = crouch;
    goal_gaitState = crouch;
    prior_gaitState = crouch;
    last_unstable_timestamp = ros::Time::now();

    m_sub_reload_config = m_nh->subscribe("/humanoid/ReloadMotionConfig", 1, &GaitStateManager::reload_gaitdata, this);
    m_pub = m_nh->advertise<dmotion::MotionInfo>("/humanoid/MotionInfo", 1);
}


GaitStateManager::~GaitStateManager() = default;

void
GaitStateManager::tick()
{
    ROS_DEBUG("GaitStateManager tick ..");
    checkNewCommand(m_cmd);
    prior_gaitState = gaitState;

    if (prior_gaitState != goal_gaitState) {
        prior_gaitState->exit();
    }

    if (prior_gaitState != goal_gaitState) {
        goal_gaitState->entry();
    }

    gaitState = goal_gaitState;

    if (rstatus->m_bStable == frontdown) {
        gaitState = setupfrontdown;
        gaitState->entry();
    } else if (rstatus->m_bStable == rightdown || rstatus->m_bStable == leftdown || rstatus->m_bStable == backdown) {
        gaitState = setupbackdown;
        gaitState->entry();
    }

    gaitState->execute();

    /* write motion share data to blackboard */
    if (!m_motion_info.lower_board_started) {
        m_motion_info.lower_board_started = true;
        m_start_time = ros::Time::now();
    }

    // FIXME(MWX): handle delta, not async!!!!!!!!!!!
    auto delta = rstatus->checkDeltaDist();
    m_delta.x = delta.m_x;
    m_delta.y = delta.m_y;
    m_delta.z = delta.m_angle;

    // TODO(corenel) check lower_board_connected in dvision
    m_motion_info.timestamp = ros::Time::now();
    ros::Duration uptime = m_motion_info.timestamp - m_start_time;
    m_motion_info.uptime = uptime.toSec();
}

void
GaitStateManager::init()
{
    ROS_INFO("GaitState Manager INIT");
    rstatus = new RobotStatus(m_nh);
    if(!m_simulation) {
        port = new transitHub(m_nh, rstatus);
    } else {
        transmit = new dtransmit::DTransmit("127.0.0.1");
    }
    robot = new HumanRobot(m_nh, port, rstatus, this);
}

void
GaitStateManager::init_allstates()
{
    GaitStateBase::set_nh(m_nh);
    walk = new GaitStateWenxi(robot);
    crouch = new GaitStateCrouch(robot);
    standup = new GaitStateStandup(robot);
    kick = new GaitStateKick(robot, this);
    goalie = new GaitStateGoalie(robot);
    setupfrontdown = new GaitStateSetupFrontDown(robot, this);
    setupbackdown = new GaitStateSetupBackDown(robot, this);
    ROS_INFO("All gait state initialised");
}

void
GaitStateManager::reload_gaitdata(const std_msgs::String::ConstPtr& msg)
{
    rstatus->initMotor();
    port->update_initdata();
    walk->loadGaitFile();
    crouch->loadGaitFile();
    standup->loadGaitFile();
    kick->loadGaitFile();
    goalie->loadGaitFile();
    setupfrontdown->loadGaitFile();
    setupbackdown->loadGaitFile();
}

void
GaitStateManager::setCmd(dmotion::ActionCommand cmd) {
    m_cmd = cmd;
}

/************************************************
 * plat ctrl
 * get plat request from behaviour blackboard
 ***********************************************/

// called in HumanRobot ... Motion Code's dependency is Horrible and Vulnerable.

void
GaitStateManager::platCtrl(double& targetYaw, double& targetPitch)
{
    // this function get called 20*30 times ps
    using dmotion::ActionCommand;

    desPitch = m_cmd.headCmd.pitch;
    desYaw = m_cmd.headCmd.yaw;

    desYaw = min(desYaw, MAX_PLAT_YAW);
    desYaw = max(desYaw, -MAX_PLAT_YAW);

    desPitch = max(desPitch, MIN_PLAT_PITCH);
    desPitch = min(desPitch, MAX_PLAT_PITCH);

    double pitchSpeed = m_cmd.headCmd.pitchSpeed;
    double yawSpeed = m_cmd.headCmd.yawSpeed;

    // set desYaw
    if (fabs(desYaw - targetYaw) < yawSpeed) {
        targetYaw = desYaw;
    } else {
        if (desYaw > targetYaw) {
            targetYaw += yawSpeed;
        } else {
            targetYaw -= yawSpeed;
        }
    }

    // set despitch
    if (fabs(desPitch - targetPitch) < pitchSpeed || targetPitch < 0) {
        targetPitch = desPitch;
    } else {
        if (desPitch > targetPitch) {
            targetPitch += pitchSpeed;
        } else {
            targetPitch -= pitchSpeed;
        }
    }

    auto eular_angle = rstatus->getEularAngle() * 180 / M_PI;

    //  auto timestamp = getCurrentTime();
    auto timestamp = ros::Time::now();

    // head protect
    auto angle = 15;

    auto current_gait = m_cmd.bodyCmd.gait_type;

    if (current_gait == BodyCommand::STANDUP || current_gait == BodyCommand::KICKLEFT || current_gait == BodyCommand::KICKRIGHT) {
        angle = 10;
    }

    if ((eular_angle.m_y - angle) < -35) {
        last_unstable_timestamp = timestamp;
        targetYaw = 0;
        targetPitch = 70;
    } else if ((eular_angle.m_y - angle) > 35) {
        last_unstable_timestamp = timestamp;
        targetYaw = 0;
        targetPitch = -70;
    } else {
        // is stable, keep for 1 second
        //     int64_t diff = timestamp - last_unstable_timestamp;
        //     if(diff < 500000) {
        //       VecPos last_plat = readFrom(motion, curPlat);
        //       targetYaw = last_plat.m_x;
        //       targetPitch = last_plat.m_y;
        //     }
    }

    // TODO(MWX): FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // get delta data
    m_motion_info.action.headCmd.pitch = targetPitch;
    m_motion_info.action.headCmd.yaw = targetYaw;
    m_motion_info.deltaData = m_delta;

    m_motion_info.stable = static_cast<int>(rstatus->m_bStable);
    m_motion_info.vy = robot->m_robotCtrl.getWalkVelY();
    m_motion_info.curPlat.x = estimated_plat.m_x;
    m_motion_info.curPlat.y = estimated_plat.m_y;

    m_motion_info.robotCtrl.x = robot->m_robotCtrl.robot_x;
    m_motion_info.robotCtrl.y = robot->m_robotCtrl.robot_x;
    m_motion_info.robotCtrl.z = robot->m_robotCtrl.robot_t;

    m_pub.publish(m_motion_info);

    if(m_simulation) {
        transmit->sendRos<dmotion::MotionInfo>(dconstant::network::robotMotionBase + m_robotId, m_motion_info);
    }
// TODO(MWX): angle feedback
// todo, magic number , maybe different with different type of dynamixel servo
// TODO(mwx): add this if required by behaviour
}

void
GaitStateManager::checkNewCommand(const dmotion::ActionCommand& request)
{
    m_cmd = request;
    switch (request.bodyCmd.gait_type) {
        case BodyCommand::WENXI:
            goal_gaitState = walk;
            goal_gaitState->m_gait_sx = request.bodyCmd.x;
            goal_gaitState->m_gait_sy = request.bodyCmd.y;
            goal_gaitState->m_gait_st = request.bodyCmd.t;
            break;
        case BodyCommand::CROUCH:
            goal_gaitState = crouch;
            break;
        case BodyCommand::STANDUP:
            goal_gaitState = standup;
            break;
        case BodyCommand::KICKLEFT:
            goal_gaitState = kick;
            // FIXME(MWX): kick side
            break;
        case BodyCommand::KICKRIGHT:
            goal_gaitState = kick;
            break;
            // FIXME(MWX): Goalie side
        case BodyCommand::GOALIELEFT:
            goal_gaitState = goalie;
            break;
        case BodyCommand::GOALIEMID:
            goal_gaitState = goalie;
            break;
        case BodyCommand::GOALIERIGHT:
            goal_gaitState = goalie;
            break;
        case BodyCommand::SETUPFRONT:
            goal_gaitState = setupfrontdown;
            break;
        case BodyCommand::SETUPBACK:
            goal_gaitState = setupbackdown;
            break;
        default:
            // would never happen
            ROS_FATAL("WRONG gait type, going to stand up");
            goal_gaitState = standup;
    }
}
