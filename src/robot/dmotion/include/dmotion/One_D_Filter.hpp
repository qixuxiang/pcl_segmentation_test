#pragma once

class One_D_Filter
{
  public:
    One_D_Filter();

    One_D_Filter(double R_);

    double update(double prediction, double measurement);

  private:
    double P;
    double Q;
    double R;
    double K;

    double estimated;
};
