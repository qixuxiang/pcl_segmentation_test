#pragma once

namespace KALMAN {
extern double T;
}

class KalmanFilter
{
  public:
    KalmanFilter();
    void estimate(double wx, double wy, double wz, double ax, double ay, double az, double time = 0.018); // time is in seconds
    double eular[2];

  private:
    double A[3][3];
    double est_last[3];
    double est_p[3][3];
};
