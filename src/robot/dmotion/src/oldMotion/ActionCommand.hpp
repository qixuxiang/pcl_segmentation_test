#pragma once

#include <iostream>
#include "fast_math.hpp"
#include "misc/types/streamable.hpp"
#include "misc/utils/logger/logger.hpp"

using namespace dancer2050;

namespace ActionCommand {
enum GaitType {
  wenxi_gait = 1,
  wenxi_gaits,
  walkrightkick,  // strategy available
  walkleftkick,   //

  standup = 10,
  crouch,  // strategy available
  kick,
  leftdownMove,   // strategy available
  rightdownMove,  // strategy available
  getup_for,      // strategy available
  getup_back,     // strategy available
                  // 5
  goalieLeft,     // strategy available       55
  goalieRight,    // strategy available
  goalieMid,      // strategy available
};

struct Body : public Streamable<Body> {
  int gaitType;

  double m_gait_sx;
  double m_gait_sy;
  double m_gait_st;
  bool rightKick_;

  MSGPACK_DEFINE_MAP(gaitType, m_gait_sx, m_gait_sy, m_gait_st, rightKick_)

  friend std::ostream& operator<<(std::ostream& out, const Body&);

  Body(double forward, double left, double turn)
      : gaitType(ActionCommand::wenxi_gait),
        m_gait_sx(forward),
        m_gait_sy(left),
        m_gait_st(turn) {
  }

  Body(GaitType gt, bool rightkick) : gaitType(gt), rightKick_(rightkick) {
    m_gait_sx = 0;
    m_gait_sy = 0;
    m_gait_st = 0;
  }

  // for crouch etc.
  Body(GaitType gt) : gaitType(gt) {
    m_gait_sx = 0;
    m_gait_sy = 0;
    m_gait_st = 0;
    rightKick_ = true;
  }

  Body() : gaitType(ActionCommand::standup) {
  }

};

/**
 * Command for controlling the head
 **/
struct Head : public Streamable<Head> {
  double yaw;
  // LEFT-RIGHT motion.
  double pitch;
  // UP-DOWN angle.
  double yawSpeed;
  // Speed of the yaw
  double pitchSpeed;
  // Speed of the pitch

  Head(double y, double p, double ys = 10, double ps = 10)
      : yaw(y), pitch(p), yawSpeed(ys), pitchSpeed(ps) {
  }

  Head() : yaw(0.0), pitch(0.0), yawSpeed(10), pitchSpeed(10) {
  }

  friend std::ostream& operator<<(std::ostream& out, const Head&);

  MSGPACK_DEFINE_MAP(yaw, pitch, yawSpeed, pitchSpeed)
};

// std::ostream &operator<<(std::ostream &out, const VecPos &pos) {
//   out << "(" << pos.m_x << "," << pos.m_y << ")";
//   return out;
// }
/**
 * Wrapper for the other action commands, makes it easier to pass them around
 **/
struct All : public Streamable<All> {
  Head head;
  Body body;

  All() : head(), body(ActionCommand::standup) {
  }

  All(Head h, Body b) : head(h), body(b) {
  }

  friend std::ostream& operator<<(std::ostream& out, const All&);

  MSGPACK_DEFINE_MAP(head, body)
};

}  // namespace ActionCommand
