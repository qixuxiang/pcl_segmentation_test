#pragma once
#include <deque>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "dmotion/VecPos.hpp"
#include "dmotion/MotionData.hpp"
#include "dmotion/MotionShareData.hpp"
#include "dmotion/GaitStateSupportLib/comEstimator.hpp"
#include "dmotion/One_D_Filter.hpp"

class RobotStatus {
 public:
  RobotStatus();
  ~RobotStatus();
  void readOptions();

  /* get infromation about robot status */
 public:
  angledataDebug getAngledata();
  GyroData getGdata();
  void setGdata(GyroData);
  void updateDeltaDist(const VecPos& D, double deltaBodyAngle);
  initdataDebug getMotorinit();
  initdataDebug getRawMotorInit();
  void updateEularAngle();
  VecPos getEularAngle();  // 弧度制
  stabilityStatus checkStableState();
  void setCompassData(CompassData temp);
  void update_compass_config(double, double, double, double, double);
  double getCurrentFieldAngle();
  CompassData getCompassData();
  comEstimator* m_comInfo;
  std::deque<VecPos> m_eular_deque;
  deltadataDebug checkDeltaDist();
  bool initMotor();
  double m_last_angle_z;

 public:
  stabilityStatus m_bStable;
  double curyaw;
  angledataDebug m_angledata;
  GyroData m_gypdata;
  offsetDebug m_offset;
  CompassData m_compassdata;
  initdataDebug m_motorini;
  initdataDebug raw_motorini;
  deltadataDebug m_deltaDist;
  double k_tmp[MOTORNUM];
  bool m_isRun;
  void updateMotor(int id, int delta);

 private:


  One_D_Filter compass_filter;
  int robotnumber;
};
