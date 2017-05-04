#pragma once

class SplineThree {
 public:
  SplineThree(int* plotx, double* ploty, int ask, int num);
  SplineThree(int* plotx, double* ploty, int ask, int num, double temp1,
              double temp2);
  ~SplineThree();

  void Calculate();
  double ReturnValue(double plotxx);

 private:
  int* x;
  double xx, *y, *a, *b, *a1, *b1, *h, *m;
  int choice;
  int n;
  double _temp1, _temp2;
};