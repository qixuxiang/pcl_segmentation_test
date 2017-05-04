#pragma once
#include <string>
#include "dmotion/MotionData.hpp"

class I_HumanRobot {
 public:
  virtual ~I_HumanRobot();

  virtual void runWalk(double tsx, double tsy, double tst) = 0;
  virtual void getAngle_serial(const RobotCtrl targetCtrl, int dataArray[], const bool isExcute) = 0;
  virtual void doTxTask(int* motionData) = 0;
  virtual double* curveCreate(double start, double mid, double last, int num) = 0;
  virtual int loadGaitFile(const std::string filename, double* data[], int colomn) = 0;
  virtual void doCrouchFromStand(const int stepnum_) = 0;
  virtual void doCrouchFromStandMotor(const int stepnum_) = 0;
  virtual void doStandFromCrouch(const int stepnum_) = 0;
  virtual void dofirstStep() = 0;
  virtual void staticEntry() = 0;
  virtual void staticExit() = 0;

  RobotCtrl m_robotCtrl;

 public:
  std::string m_robot_name;
  int m_robot_number;

  bool m_leftkick_flag;
  bool m_rightkick_flag;
  double m_motor_k[MOTORNUM];
  int m_motor_zf[MOTORNUM];
  int m_motor_lb[MOTORNUM];
  int m_motor_ub[MOTORNUM];
  double m_AnkleH_mid_l;   // the stepheight up
  double m_AnkleH_last_l;  // the stepheight down
  double m_AnkleH_mid_r;
  double m_AnkleH_last_r;

 public: /* to do be private */
         /* options */
         // int m_oldTurning;
         // double m_stepK;
         // /* compensate the turning angle */
         // double m_theta_AnticlockZ,m_theta_ClockwiseF;

  // /**
  //  * gait data
  //  **/
  // double m_g;
  // /* sample time */
  // double m_dt;
  // /* the distance between two ankles (id 6 and id 13 motors) */
  // double m_ankle_distance;
  // /* the stepheight up */
  // double m_AnkleH_mid_l;
  // /* the stepheight down */
  // double m_AnkleH_last_l;
  // double m_AnkleH_mid_r;
  // double m_AnkleH_last_r;
  // int    m_stepnum;
  // double m_yzmp;
  // //---------                 ---------
  // //|       |        x        |       |
  // //|   .   |   .    |        |   .   |
  // //|   |   |   |    |        |   |   |
  // //----|----   |y<---        ----|----
  // //    |-------|                 |
  // //    | m_yzmp                  |
  // //    |-------------------------|
  // //          m_ankle_distance

  // double m_cm_r;
  // double m_cm_p;
  // double m_cm_y;
  // double m_cm_dx;
  // double m_cm_dy;
  // /* m_cm_dx_k*forwardmovelength = cm offset in x's direction */
  // double m_cm_dx_k;
  // /* m_cm_dy_lk*leftmovelength = cm offset in y's direction */
  // double m_cm_dy_lk;
  // double m_cm_dy_rk;
  // double m_percent_x;
  // double m_kickpercent;
  // /* like center of mass */
  // double m_hipHeight;
  // /* the offset when step */
  // double m_stepZero;
};
