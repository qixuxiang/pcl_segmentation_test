#include "dmotion/One_D_Filter.hpp"
#include <iostream>
using namespace std;

One_D_Filter::One_D_Filter() {
  P = 1;
  Q = 1;
  R = 1000;
  K = 1;
  estimated = 0;
}

One_D_Filter::One_D_Filter(double R_) {
  P = 1;
  Q = 1;
  R = R_;
  K = 1;
  estimated = 0;
}

double One_D_Filter::update(double prediction, double measurement) {
  P += Q;
  K = P / (P + R);
  P -= K * P;
  estimated += K * (measurement - estimated) + prediction;

  return estimated;
}
