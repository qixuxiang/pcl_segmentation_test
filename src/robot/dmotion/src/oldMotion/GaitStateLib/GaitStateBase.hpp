#pragma once
#include "misc/utils/logger/logger.hpp"
#include "motion/GaitStateType.hpp"
#include "motion/ActionCommand.hpp"
#include "motion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "motion/MotionConfig.hpp"
#include <string>
using namespace dancer2050;

using namespace std;
using namespace ActionCommand;

class GaitStateBase {
 public:
  virtual void execute() = 0;
  virtual void entry() {}
  virtual void exit() {}
  GaitStateType getStateType() {return type;}
  GaitType      getGaitType() {return m_gait_type;}

  bool operator==(const GaitStateType type);
  bool operator!=(const GaitStateType type);
  virtual ~GaitStateBase(){};

  double m_gait_sx;
  double m_gait_sy;
  double m_gait_st;
  double m_gait_angle;
  ActionCommand::GaitType  m_gait_type;

  int    m_gait_index;
  bool   m_right_kick;

  int stepNum;
  int m_robot_number;
  //ActionCommand::StaicGait m_static_gait;

  double m_setupside_ankle_theta;
  double m_setupside_arm_theta;
  double m_goalie_theta;
  bool m_goalie_bool;

  void readOptions();

  virtual void loadGaitFile();

 protected:
  /* GaitStateBase Constructor */
  GaitStateBase(int layer, GaitStateType type, I_HumanRobot* robot)
      : m_gait_sx(0),
        m_gait_sy(0),
        m_gait_st(0),
        m_gait_angle(0),
        m_gait_type(crouch),
        m_gait_index(0),
        m_right_kick(true),
        layer(layer),
        type(type),
        robot(robot),
        parent_gt(NULL) {
          readOptions();
        }

  const int layer;
  const GaitStateType type;
  I_HumanRobot* robot;
  GaitStateBase* parent_gt;
};
