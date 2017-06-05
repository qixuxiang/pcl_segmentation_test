/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:51:35+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: localization.hpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:52:44+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/kalman.hpp"
#include "dvision/line_segment.hpp"
#include "dvision/parameters.hpp"
#include "dvision/projection.hpp"
#include "dvision/utils.hpp"
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>

namespace dvision {
class Localization
{
  public:
    explicit Localization();
    ~Localization();
    void init();

  private:
};
} // namespace dvision
