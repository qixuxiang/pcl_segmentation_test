#pragma once
#include <deque>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "VecPos.hpp"
#include "MotionData.hpp"
#include "MotionShareData.hpp"
#include "ActionCommand.hpp"

#include "GaitStateSupportLib/comEstimator.hpp"
#include "One_D_Filter.hpp"

namespace compass_config {
  extern double x0;
  extern double y0;
  extern double angle0;
  extern double phi;
  extern double ratio;
}

double getFieldAngle(double x, double y, double x0, double y0, double angle0, double phi, double ratio);
void update_compass_config(double x0, double y0, double angle0, double phi0, double ratio);

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
  ActionCommand::All all;
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
  void saveMotor();

 private:


  One_D_Filter compass_filter;
  int robotnumber;
};
