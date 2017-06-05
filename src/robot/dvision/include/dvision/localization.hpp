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
    LineContainer(LineSegment _lineTransformed, LineType _type);
    LineSegment lineTransformed;
    LineType type;
};

class FeatureContainer
{
  public:
    FeatureContainer(cv::Point2f _position, std::string _type);
    cv::Point2f position;
    std::string type;
};

class Localization
{
  public:
    explicit Localization();
    ~Localization();

    g2o::SparseOptimizer optimizer;
    double A;
    double B;
    double E;
    double F;
    double G;
    double H;
    double D;
    double I;
    double A2;
    double B2;
    double E2;
    double F2;
    double G2;
    double H2;
    double D2;
    double I2;

    bool Init();
    bool Update(Projection& projection);
    bool Calculate(std::vector<LineSegment>& clusteredLines,
                   const bool& circleDetected,
                   const cv::Point2f& FieldHullRealCenter,
                   const std::vector<cv::Point2f>& FieldHullReal,
                   const cv::Point2d& resultCircleRotated,
                   const std::vector<cv::Point2f>& goalPosition,
                   std::vector<LineContainer>& AllLines,
                   std::vector<FeatureContainer>& AllFeatures);

    // graph
    void InitG2OGraph();
    void updateVertexIdx();
    bool addObservation(cv::Point2d observation, const double& xFasher, const double& yFasher, const LandmarkType& type);

    // getter
    cv::Point2f GetBall();
    cv::Point3d GetLocalization();
    cv::Point2d GetOdometryFromLastGet();

    // setter
    void SetBall(const cv::Point2f& _in);

  private:
    cv::Point2d location;
    cv::Point2d locationKalman;
    KalmanFilterC kalmanI;
    Projection* _cameraProjections;
    float lastOdomX, lastOdomY;
    cv::Point3d globalPos;
    cv::Point3d lastOdom;
    uint32_t lastOdomID;
    cv::Point2f ballPos;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    g2o::BlockSolverX* blockSolver;
    g2o::OptimizationAlgorithmLevenberg* optimizationAlgorithm;
    int CurrentVertexId;
    int PreviousVertexId;
    int LandmarkCount;
    cv::Point2d odomLastGet;
    bool atLeastOneObservation;
    long unsigned int nodeCounter;

    double GetUpdateCoef(const double& coef, const cv::Point2f& point);
    double GetUpdateCoef(const double& coef, LineSegment line);
};
} // namespace dvision
