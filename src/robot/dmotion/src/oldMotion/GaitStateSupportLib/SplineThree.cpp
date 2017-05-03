#include "SplineThree.hpp"
#include <cmath>

SplineThree::SplineThree(int *plotx, double *ploty, int ask, int num) {
  x = plotx;
  y = ploty;
  choice = ask;
  n = num;
  a = new double[n];
  b = new double[n];
  a1 = new double[n];
  b1 = new double[n];
  h = new double[n - 1];
  m = new double[n + 1];
  _temp1 = 0;
  _temp2 = 0;
}
SplineThree::SplineThree(int *plotx, double *ploty, int ask, int num,
                         double temp1, double temp2) {
  x = plotx;
  y = ploty;
  choice = ask;
  n = num;
  _temp1 = temp1;
  _temp2 = temp2;
  a = new double[n];
  b = new double[n];
  a1 = new double[n];
  b1 = new double[n];
  h = new double[n - 1];
  m = new double[n + 1];
}

SplineThree::~SplineThree(){}

void SplineThree::Calculate() {
  switch (choice) {
    case 1: {
      a[0] = 0;
      a[n - 1] = 1;
      b[0] = 2 * _temp1;
      b[n - 1] = 2 * _temp2;
      break;
    }
    case 2: {
      a[0] = 1;
      a[n - 1] = 0;
      b[0] = 3 / h[0] * (y[1] - y[0]);
      b[n - 1] = 3 / h[n - 2] * (y[n - 1] - y[n - 2]);
      break;
    }
  }
  for (int k = 1; k < n - 1; k++) {
    a[k] = h[k - 1] / (h[k - 1] + h[k]);
    b[k] = 3 * ((1 - a[k]) / h[k - 1] * (y[k] - y[k - 1]) +
                a[k] / h[k] * (y[k + 1] - y[k]));
  }
  a1[0] = -a[0] / 2;
  b1[0] = b[0] / 2;
  for (int l = 1; l < n; l++) {
    a1[l] = -a[l] / (2 + (1 - a[l]) * a1[l - 1]);
    b1[l] = (b[l] - (1 - a[l]) * b1[l - 1]) / (2 + (1 - a[l]) * a1[l - 1]);
  }
  m[n] = 0;
  for (int j = n - 1; j >= 0; j--) {
    m[j] = a1[j] * m[j + 1] + b1[j];
  }
}

double SplineThree::ReturnValue(double plotxx) {
  xx = plotxx;
  double output = 0;
  for (int k = 0; k < n - 1; k++) {
    if (x[k] <= xx && x[k + 1] > xx) {
      output =
          (1 + 2 * (xx - x[k]) / (x[k + 1] - x[k])) *
              pow(((xx - x[k + 1]) / (x[k] - x[k + 1])), 2) * y[k] +
          (1 + 2 * (xx - x[k + 1]) / (x[k] - x[k + 1])) *
              pow(((xx - x[k]) / (x[k + 1] - x[k])), 2) * y[k + 1] +
          (xx - x[k]) * pow(((xx - x[k + 1]) / (x[k] - x[k + 1])), 2) * m[k] +
          (xx - x[k + 1]) * pow(((xx - x[k]) / (x[k + 1] - x[k])), 2) *
              m[k + 1];
      break;
    }
  }
  return output;
}
