#include "GaitStateManager.hpp"
#include <algorithm>
#include "GaitStateLib/AllGaitState.hpp"
#include "GaitStateSupportLib/HumanRobot.hpp"
#include "misc/utils/logger/logger.hpp"
#include "misc/utils/timer/timer.hpp"
#include "misc/node/node.hpp"
#include "misc/types/vector2.hpp"
using namespace std;
using namespace dancer2050;

extern ActionCommand::All behave_req;  // updated in main.cpp
extern Node mynode;

namespace AC = ActionCommand;
namespace {
double angle_pitch = 15;
}

// void set_start_angle_pitch(double angle){
//   angle_pitch = angle;
// }

GaitStateManager::GaitStateManager() {
  LOG(INFO, "    [ Constructing GaitStateManager");
  init();
  init_allstates();
  // !?!?
  // set_start_angle_pitch((bb->config)["robot.angle_pitch"].as<double>());
  // head_k = (bb->config)["robot.head_k"].as<double>();

  if (head_k >= 1) {
    head_k = 0.5;
  }

  LOG(INFO, "    GaitStateManager constructed ]");

  gaitState = crouch;
  goal_gaitState = crouch;
  prior_gaitState = crouch;
  last_unstable_timestamp = 0;
}

GaitStateManager::~GaitStateManager() {
  LOG(INFO, "GaitStateManager destoried");
}

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
  rstatus = new RobotStatus();
  port = new transitHub(rstatus);
  robot = new HumanRobot(port, rstatus, this);
}

void GaitStateManager::init_allstates() {
  LOG(INFO, "            Initializing all gait states");
  walk = new GaitStateWenxi(robot);
  crouch = new GaitStateCrouch(robot);
  standup = new GaitStateStandup(robot);
  kick = new GaitStateKick(robot, this);
  goalieleft = new GaitStateGoalieleft(robot);
  goalieright = new GaitStateGoalieright(robot);
  goaliemid = new GaitStateGoaliemid(robot);

  setupfrontdown = new GaitStateSetupFrontDown(robot, this);
  setupbackdown = new GaitStateSetupBackDown(robot, this);
  setupleftdown = new GaitStateSetupLeftDown(robot, this);
  setuprightdown = new GaitStateSetupRightDown(robot, this);

  walkleftkick = new GaitStateWalkLeftkick(robot);
  walkrightkick = new GaitStateWalkRightkick(robot);
  LOG(INFO, "            All gait states initialized");
}

void GaitStateManager::reload_gaitdata() {
  rstatus->initMotor();
  port->update_initdata();

  walk->loadGaitFile();
  crouch->loadGaitFile();
  standup->loadGaitFile();
  kick->loadGaitFile();
  goalieleft->loadGaitFile();
  goalieright->loadGaitFile();
  goaliemid->loadGaitFile();
  setupfrontdown->loadGaitFile();
  setupbackdown->loadGaitFile();
  setupleftdown->loadGaitFile();
  setuprightdown->loadGaitFile();
  walkleftkick->loadGaitFile();
  walkrightkick->loadGaitFile();
}

/************************************************
 * plat ctrl
 * get plat request from behaviour blackboard
 ***********************************************/

// called in HumanRobot ... Motion Code's dependency is Horrible and Vulnerable.

void GaitStateManager::platCtrl(double &targetYaw, double &targetPitch) {
  // this function get called 20*30 times ps
  // ActionCommand::Head head = readFrom(behaviour, actions.head);
  ActionCommand::Head head = behave_req.head;
  // LOG(DEBUG, head);

  desYaw = head.yaw;
  desPitch = head.pitch;

  desYaw = min(desYaw, MAX_PLAT_YAW);
  desYaw = max(desYaw, -MAX_PLAT_YAW);

  desPitch = max(desPitch, MIN_PLAT_PITCH);
  desPitch = min(desPitch, MAX_PLAT_PITCH);

  double yawSpeed = head.yawSpeed;
  double pitchSpeed = head.pitchSpeed;

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

  auto timestamp = getCurrentTime();

  // head protect
  auto angle = angle_pitch;

  auto current_gait = behave_req.body.gaitType;

  if (current_gait == ActionCommand::standup ||
      current_gait == ActionCommand::kick) {
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
    // int64_t diff = timestamp - last_unstable_timestamp;
    // if(diff < 500000) {
    //   VecPos last_plat = readFrom(motion, curPlat);
    //   targetYaw = last_plat.m_x;
    //   targetPitch = last_plat.m_y;
    // }
  }

  //  head.yaw = targetYaw;
  //  head.pitch = targetPitch;
  // todo, magic number , maybe different with different type of dynamixel servo
  // double K = head_k;
  //  K = K * pitchSpeed;
  // estimated_plat.m_x += K * (desYaw - estimated_plat.m_x);
  // estimated_plat.m_y += K * (desPitch - estimated_plat.m_y);
//  cout << estimated_plat << endl;

//  VecPos tmpPlat(targetYaw, targetPitch);  // todo, check delay

// TODO(mwx): add this if required by behaviour

  //!?!?!?!!?!?!??!?!?!?!?!?
  // index v=123 h
#if 1
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

void GaitStateManager::checkNewCommand(ActionCommand::All &request) {
  switch (request.body.gaitType) {
    case AC::wenxi_gait:
      goal_gaitState = walk;
      goal_gaitState->m_gait_sx = request.body.m_gait_sx;
      goal_gaitState->m_gait_sy = request.body.m_gait_sy;
      goal_gaitState->m_gait_st = request.body.m_gait_st;

      break;
    case AC::wenxi_gaits:
      goal_gaitState = walk;
      // goal_gaitState->m_gait_vs = request.body.gait_vs_;
      break;
    case AC::crouch:
      goal_gaitState = crouch;
      break;
    case AC::standup:
      // std::cout << "checkNewCommand stand " << std::endl;
      goal_gaitState = standup;
      break;

    case AC::kick:
      goal_gaitState = kick;
      goal_gaitState->m_right_kick = request.body.rightKick_;
      break;
    case AC::goalieLeft:
      goal_gaitState = goalieleft;
      break;
    case AC::goalieRight:
      goal_gaitState = goalieright;
      break;
    case AC::goalieMid:
      goal_gaitState = goaliemid;
      break;
    case AC::getup_for:
      goal_gaitState = setupfrontdown;
      break;
    case AC::getup_back:
      goal_gaitState = setupbackdown;
      break;
    case AC::leftdownMove:
      goal_gaitState = setupleftdown;
      break;
    case AC::rightdownMove:
      goal_gaitState = setuprightdown;
      break;
    case AC::walkleftkick:
      goal_gaitState = walkleftkick;
      break;
    case AC::walkrightkick:
      goal_gaitState = walkrightkick;
      break;
    default:
      LOG(FATAL, "GaitStateManager checkNewCommand Unknown state");
  }
}
