#include "ActionCommand.hpp"


namespace ActionCommand {
std::ostream& operator<<(std::ostream& out, const Body& body) {
  out << "{ gaitType: " << body.gaitType
    << " m_gait_sx: " << body.m_gait_sx
    << " m_gait_sy: " << body.m_gait_sy
    << " m_gait_st: " << body.m_gait_st
   << " rightKick_: " << body.rightKick_ << " }";

   return out;
}

std::ostream& operator<<(std::ostream& out, const Head& head) {
  out << "{ yaw: " << head.yaw
  <<  " pitch: " << head.pitch
 << " yawSpeed:" << head.yawSpeed
 << " pitchSpeed: " << head.pitchSpeed << " }";
 return out;
}

std::ostream& operator<<(std::ostream& out, const All& all) {
  out << "Head: " << all.head << '\n'
     << " body: " << all.body;
  return out;
}
} // namespace dancer2050
