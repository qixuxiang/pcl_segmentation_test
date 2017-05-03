#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <stdexcept>

#define UNKNOW_DOUBLE_VALUE (-9999)
#define INF_INT 99999
#define EPSILN  0.0001;

inline double AngleNormalization(double angle) {
  double a = angle;
  while (a > 180) {
    a -= 360;
  }
  while (a <= -180) {
    a += 360;
  }
  return a;
}

inline double degrees(double rad) {
  return rad * 180 / M_PI;
}

inline double radians(double deg) {
  return deg * M_PI / 180;
}

/**
 * safe way to judge aangle equal
 **/
inline bool angleEqual(double a, double b) {
  return std::abs(a - b) < EPSILN;
}

inline bool double_equal(double a, double b) {
  auto diff = a - b;
  return diff < 0.0001 && diff > -0.0001;
}

inline double getPieceWise(const std::vector<double> &x,
                           const std::vector<double> &y, const double wx) {
  if (x.size() != y.size())
    throw std::runtime_error("getPieceWise function , the vector size not same ");
  if (!x.size())
    throw std::runtime_error("getPieceWise function , the vector size empty ");
  int size = x.size();
  double returnValue = y[1];

  if (wx < x[0])
    return y[0];
  if (wx > x[size - 1])
    return y[size - 1];

  //first calculate k
  // double k[size - 1];
  double* k = new double[size - 1];
  for (int i = 0; i < size - 1; i++) {
    if (std::abs(x[i + 1] - x[i]) < 0.0001)
      k[i] = 0;
    else
      k[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    std::cout<< k[i] <<std::endl;

  }
  for (int i = 0; i < size; i++) {
    if (wx < x[i]) // i = 0 will never happan
    {
      returnValue =
        y[i - 1] + k[i - 1] * (wx - x[i - 1]); //RobotPara::mid_theta_amend + k*(tsx - RobotPara::mid_x_max);
      break;
    }
  }
  delete[] k;
  return returnValue;

}
