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
#include <string>
#include <vector>

namespace dvision {
enum LineType
{
    HorUndef,
    HorCenter,
    HorGoal,
    HorGoalNear,
    VerUndef,
    VerLeft,
    VerRight,
    VerLeftNear,
    VerRightNear,
    VerLeftT,
    VerRightT,
    LTRES
};

enum LandmarkType
{
    CenterL = 0,
    RightL,
    FrontL,
    LeftL,
    BackL
};

const std::string LineTypeName[LTRES] = { "HorUndef", "HorCenter", "HorGoal", "HorGoalNear", "VerUndef", "VerLeft", "VerRight", "VerLeftNear", "VerRightNear", "VerLeftT", "VerRightT" };

class LineContainer
{
  public:
    LineContainer(LineSegment lineTransformed, LineType type);
    LineSegment line_transformed_;
    LineType type_;
};

class FeatureContainer
{
  public:
    FeatureContainer(cv::Point2f position, std::string type);
    cv::Point2f position_;
    std::string type_;
};

class Localization
{
  public:
    explicit Localization();
    ~Localization();

    g2o::SparseOptimizer optimizer;

    bool Init();
    bool Update(Projection& projection);
    bool Calculate(std::vector<LineSegment>& clustered_lines,
                   const bool& circle_detected,
                   const cv::Point2f& field_hull_real_center,
                   const std::vector<cv::Point2f>& field_hull_real,
                   const cv::Point2d& result_circle_rotated,
                   const std::vector<cv::Point2f>& goal_position,
                   std::vector<LineContainer>& all_lines,
                   std::vector<FeatureContainer>& all_features);

    // graph
    void InitG2OGraph();
    void UpdateVertexIdx();
    bool AddObservation(cv::Point2d observation, const double& x_fasher, const double& y_fasher, const LandmarkType& type);

    // getter
    cv::Point2f ball();
    cv::Point3d location();
    cv::Point2d odom_last_get();

    // setter
    void ball(const cv::Point2f& _in);

  private:
    double A_;
    double B_;
    double E_;
    double F_;
    double G_;
    double H_;
    double D_;
    double I_;
    double A2_;
    double B2_;
    double E2_;
    double F2_;
    double G2_;
    double H2_;
    double D2_;
    double I2_;
    cv::Point2d location_;
    cv::Point2d location_kalman_;
    KalmanFilterC kalmanI_;
    Projection* camera_projections_;
    float last_odom_x_, last_odom_y_;
    cv::Point3d global_pos_;
    cv::Point3d last_odom_;
    uint32_t last_odom_id_;
    cv::Point2f ball_pos_;
    g2o::BlockSolverX::LinearSolverType* linear_solver_;
    g2o::BlockSolverX* block_solver_;
    g2o::OptimizationAlgorithmLevenberg* opt_algo_;
    int current_vertex_id_;
    int previous_vertex_id_;
    int landmark_count_;
    cv::Point2d odom_last_get_;
    bool at_least_one_observation_;
    long unsigned int node_counter_;

    double GetUpdateCoef(const double& coef, const cv::Point2f& point);
    double GetUpdateCoef(const double& coef, LineSegment line);
};
} // namespace dvision
