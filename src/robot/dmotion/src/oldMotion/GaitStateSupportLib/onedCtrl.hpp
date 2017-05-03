#pragma once

// brief
//一维轨迹的生成类，用来规划摆动腿的生成轨迹，输入vmax和amax分别代表了最大速度和最大加速度。
//原理详情可以参考《全方位移动机器人运动控制及规划》.吴永海. 4.3 1D轨迹规划
class onedCtrl {
 public:
  onedCtrl(double vmax, double amax);
  ~onedCtrl(void);
  void oned_analysis(double x_out[], const double x_start[],
                     const double x_target[], const int num_left);

 private:
  double m_vmax;
  double m_amax;
  // double m_x_start[2];
  // double m_x_target[2];
  // double m_x_out[2];
  // int m_num_left;
};
