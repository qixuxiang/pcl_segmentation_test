#pragma once
#include "kalman_filter.hpp"

class comEstimator {
 public:
  comEstimator();
  void estimateComY(double bodyAngleY, int* jointAngle);

 public:
  KalmanFilter* m_filter;

  double angle_thres;
  double comX;
  double comY;
};
