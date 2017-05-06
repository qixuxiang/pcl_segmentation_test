#include "dmotion/GaitStateManager.hpp"
#include "dmotion/GaitStateLib/AllGaitState.hpp"
#include "dmotion/GaitStateSupportLib/HumanRobot.hpp"
#include "dmotion/MotionData.hpp"

using namespace std;

GaitStateManager::GaitStateManager(ros::NodeHandle* nh) : m_nh(nh) {
  init();
  init_allstates();
  gaitState = crouch;
  goal_gaitState = crouch;
  prior_gaitState = crouch;
  last_unstable_timestamp = ros::Time::now();
}

GaitStateManager::~GaitStateManager() = default;

void GaitStateManager::tick() {
  ROS_INFO("GaitStateManager tick ..");
  prior_gaitState = gaitState;

  if (prior_gaitState != goal_gaitState) {
    prior_gaitState->exit();
  }

  if (prior_gaitState != goal_gaitState) {
    goal_gaitState->entry();
  }

  gaitState = goal_gaitState;

  // if (RobotPara::getup_bool) { // Why we need this ???
  if (true) {
    if (rstatus->m_bStable == frontdown) {
      gaitState = setupfrontdown;
      gaitState->entry();
    } else if (rstatus->m_bStable == rightdown
            || rstatus->m_bStable == leftdown
            || rstatus->m_bStable == backdown) {
      gaitState = setupbackdown;
      gaitState->entry();
    }
  }

  gaitState->execute();
  /* write motion share data to blackboard */

  // robot->m_leftkick_flag = readFrom(behaviour, kick);
  // robot->m_rightkick_flag = readFrom(behaviour, kick);
}

void GaitStateManager::init() {
  ROS_INFO("GaitState Manager INIT");

  rstatus = new RobotStatus(m_nh);
  port = new transitHub(m_nh, rstatus);
  robot = new HumanRobot(m_nh, port, rstatus, this);
}

void GaitStateManager::init_allstates() {
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

void GaitStateManager::reload_gaitdata() {
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

/************************************************
 * plat ctrl
 * get plat request from behaviour blackboard
 ***********************************************/

// called in HumanRobot ... Motion Code's dependency is Horrible and Vulnerable.

void GaitStateManager::platCtrl(double &targetYaw, double &targetPitch) {
  // this function get called 20*30 times ps
  using dmotion::ActionCmd;
  auto head = m_cmd.cmd_head; // row pitch yall

  desPitch = head.y;
  desYaw = head.z;

  desYaw = min(desYaw, MAX_PLAT_YAW);
  desYaw = max(desYaw, -MAX_PLAT_YAW);

  desPitch = max(desPitch, MIN_PLAT_PITCH);
  desPitch = min(desPitch, MAX_PLAT_PITCH);

  auto& speed = m_cmd.cmd_head_speed;
  double pitchSpeed = speed.y;
  double yawSpeed = speed.z;

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
  auto angle = 15; // TODO(MWX): not implemented

  auto current_gait = m_cmd.gait_type;

  if (current_gait == ActionCmd::STANDUP ||
      current_gait == ActionCmd::KICK) {
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

  // TODO(MWX): angle feedback
  // todo, magic number , maybe different with different type of dynamixel servo
// TODO(mwx): add this if required by behaviour
#if 0
    Vector2 plat(targetYaw, targetPitch);
    mynode.publish("plat", plat);
    // writeTo(motion, curPlat, estimated_plat);
    // rstatus->curYaw = targetYaw;
    // writeTo(motion, gyro, rstatus->getGdata());
    // mynode.publish("gyro", rstatus->getGdata());
    // deltadataDebug tmpDelta = rstatus->checkDeltaDist();
    // // deltadataDebug tmpD = readFrom(motion, deltaData);
    // tmpD.m_x += tmpDelta.m_x;
    // tmpD.m_y += tmpDelta.m_y;
    // tmpD.m_angle += tmpDelta.m_angle;
    // writeTo(motion, deltaData, tmpD);
    // writeTo(motion, stable, rstatus->m_bStable);
    // writeTo(motion, eular, rstatus->getEularAngle());
    // writeTo(motion, robotCtrl, robot->m_robotCtrl);
    // writeTo(motion, vy, robot->m_robotCtrl.getWalkVelY());
#endif
}

void GaitStateManager::checkNewCommand(const dmotion::ActionCmd &request) {
  m_cmd = request;
  using dmotion::ActionCmd;
  switch (request.gait_type) {
    case ActionCmd::WENXI:
      goal_gaitState = walk;
      goal_gaitState->m_gait_sx = request.cmd_vel.linear.x;
      goal_gaitState->m_gait_sy = request.cmd_vel.linear.y;
      goal_gaitState->m_gait_st = request.cmd_vel.angular.z;
      ROS_DEBUG("WENXI (%lf, %lf, %dlf)", goal_gaitState->m_gait_sx, goal_gaitState->m_gait_sy, goal_gaitState->m_gait_st);
      break;
    case ActionCmd::CROUCH:
      goal_gaitState = crouch;
      break;
    case ActionCmd::STANDUP:
      goal_gaitState = standup;
      break;
    case ActionCmd::KICK:
      goal_gaitState = kick;
      goal_gaitState->m_kick_side = request.kick_side;
      break;
    case ActionCmd::GOALIE:
      goal_gaitState = goalie;
      break;
    case ActionCmd::SETUPFRONT:
      goal_gaitState = setupfrontdown;
      break;
    case ActionCmd::SETUPBACK:
      goal_gaitState = setupbackdown;
      break;
    default:
      // would never happen
      ROS_FATAL("WRONG gait type, going to stand up");
      goal_gaitState = standup;
  }
}
