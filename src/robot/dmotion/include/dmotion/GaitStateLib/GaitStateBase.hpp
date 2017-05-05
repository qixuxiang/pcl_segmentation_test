#pragma once
#include "dmotion/ActionCmd.h"
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "dmotion/GaitStateType.hpp"
#include "dmotion/ActionCmd.h"
#include <ros/ros.h>
#include <string>
//using dmotion::ActionCmd::gait_type;

class GaitStateBase {
 public:
  virtual void execute() = 0;
  virtual void entry() {}
  virtual void exit() {}

  bool operator==(const GaitStateType type);
  bool operator!=(const GaitStateType type);
  virtual ~GaitStateBase(){};

  double m_gait_sx;
  double m_gait_sy;
  double m_gait_st;
  double m_gait_angle;
//  bool   m_right_kick;
  dmotion::ActionCmd::_kick_side_type m_kick_side;


  int stepNum;
  int m_robot_number;
  double m_setupside_ankle_theta;
  double m_setupside_arm_theta;
  double m_goalie_theta;
  bool m_goalie_bool;
  static inline void set_nh(ros::NodeHandle* nh) { m_nh = nh; }

  void readOptions();
  virtual void loadGaitFile();

 protected:
  GaitStateBase(GaitStateType type, I_HumanRobot* robot)
      : m_gait_sx(0),
        m_gait_sy(0),
        m_gait_st(0),
        m_gait_angle(0),
        m_kick_side(dmotion::ActionCmd::LEFT_KICK),
        type(type),
        robot(robot) {
          readOptions();

    //  get_val(robot["left_y_max"], length_max_l);
//  get_val(robot["right_y_max"], length_max_r);
//  get_val(robot["top_x_max"], length_max_f);
//  get_val(robot["back_x_max"], length_max_b);
        }

  const GaitStateType type;
  I_HumanRobot* robot;
  static ros::NodeHandle* m_nh;

  double length_max_l;
  double length_max_r;
  double length_max_f;
  double length_max_b;
};
