#include "dmotion/GaitStateManager.hpp"
#include "dmotion/GaitStateLib/AllGaitState.hpp"
#include "dmotion/GaitStateSupportLib/HumanRobot.hpp"

using namespace std;

GaitStateManager::GaitStateManager(ros::NodeHandle* nh) : m_nh(nh) {
  init();
  init_allstates();
  gaitState = crouch;
  goal_gaitState = crouch;
  prior_gaitState = crouch;
  last_unstable_timestamp = 0;
}

GaitStateManager::~GaitStateManager() = default;

void GaitStateManager::tick() {
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
  ROS_ERROR("Not implemented");
  // this function get called 20*30 times ps
  // ActionCommand::Head head = readFrom(behaviour, actions.head);

//  ActionCommand::Head head = behave_req.head;
//  dmotion::ActionCmd::_cmd_head_type head =

//  desYaw = head.yaw;
//  desPitch = head.pitch;
//
//  desYaw = min(desYaw, MAX_PLAT_YAW);
//  desYaw = max(desYaw, -MAX_PLAT_YAW);
//
//  desPitch = max(desPitch, MIN_PLAT_PITCH);
//  desPitch = min(desPitch, MAX_PLAT_PITCH);
//
//  double yawSpeed = head.yawSpeed;
//  double pitchSpeed = head.pitchSpeed;
//
//  // set desYaw
//  if (fabs(desYaw - targetYaw) < yawSpeed) {
//    targetYaw = desYaw;
//  } else {
//    if (desYaw > targetYaw) {
//      targetYaw += yawSpeed;
//    } else {
//      targetYaw -= yawSpeed;
//    }
//  }
//
//  // set despitch
//  if (fabs(desPitch - targetPitch) < pitchSpeed || targetPitch < 0) {
//    targetPitch = desPitch;
//  } else {
//    if (desPitch > targetPitch) {
//      targetPitch += pitchSpeed;
//    } else {
//      targetPitch -= pitchSpeed;
//    }
//  }
//
//  auto eular_angle = rstatus->getEularAngle() * 180 / M_PI;
//
//  auto timestamp = getCurrentTime();
//
//  // head protect
//  auto angle = angle_pitch;
//
//  auto current_gait = behave_req.body.gaitType;
//
//  if (current_gait == ActionCommand::standup ||
//      current_gait == ActionCommand::kick) {
//    angle = 10;
//  }
//
//  if ((eular_angle.m_y - angle) < -35) {
//    last_unstable_timestamp = timestamp;
//    targetYaw = 0;
//    targetPitch = 70;
//  } else if ((eular_angle.m_y - angle) > 35) {
//    last_unstable_timestamp = timestamp;
//    targetYaw = 0;
//    targetPitch = -70;
//  } else {
//    // is stable, keep for 1 second
//    // int64_t diff = timestamp - last_unstable_timestamp;
//    // if(diff < 500000) {
//    //   VecPos last_plat = readFrom(motion, curPlat);
//    //   targetYaw = last_plat.m_x;
//    //   targetPitch = last_plat.m_y;
//    // }
//  }
//
//  //  head.yaw = targetYaw;
//  //  head.pitch = targetPitch;
//  // todo, magic number , maybe different with different type of dynamixel servo
//  // double K = head_k;
//  //  K = K * pitchSpeed;
//  // estimated_plat.m_x += K * (desYaw - estimated_plat.m_x);
//  // estimated_plat.m_y += K * (desPitch - estimated_plat.m_y);
////  cout << estimated_plat << endl;
//
////  VecPos tmpPlat(targetYaw, targetPitch);  // todo, check delay
//
//// TODO(mwx): add this if required by behaviour
//
//  //!?!?!?!!?!?!??!?!?!?!?!?
//  // index v=123 h
//#if 1
//    Vector2 plat(targetYaw, targetPitch);
//
//    mynode.publish("plat", plat);
//
//    // writeTo(motion, curPlat, estimated_plat);
//    // rstatus->curYaw = targetYaw;
//
//    // writeTo(motion, gyro, rstatus->getGdata());
//
//    // mynode.publish("gyro", rstatus->getGdata());
//
//    // deltadataDebug tmpDelta = rstatus->checkDeltaDist();
//
//    // // deltadataDebug tmpD = readFrom(motion, deltaData);
//    // tmpD.m_x += tmpDelta.m_x;
//    // tmpD.m_y += tmpDelta.m_y;
//    // tmpD.m_angle += tmpDelta.m_angle;
//
//    // writeTo(motion, deltaData, tmpD);
//
//    // writeTo(motion, stable, rstatus->m_bStable);
//
//    // writeTo(motion, eular, rstatus->getEularAngle());
//    // writeTo(motion, robotCtrl, robot->m_robotCtrl);
//    // writeTo(motion, vy, robot->m_robotCtrl.getWalkVelY());
//
//#endif
}

void GaitStateManager::checkNewCommand(const dmotion::ActionCmd &request) {
  using dmotion::ActionCmd;
  switch (request.gait_type) {
    case ActionCmd::WENXI:
      goal_gaitState = walk;
      goal_gaitState->m_gait_sx = request.cmd_vel.linear.x;
      goal_gaitState->m_gait_sy = request.cmd_vel.linear.y;
      goal_gaitState->m_gait_st = request.cmd_vel.angular.z;
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
