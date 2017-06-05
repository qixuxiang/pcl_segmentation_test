/**
 * @Author: Yusu Pan <corenel>
 * @Date:   2017-06-02T19:51:24+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: localization.cpp
 * @Last modified by:   corenel
 * @Last modified time: 2017-06-02T19:52:16+08:00
 * @Copyright: ZJUDancer
 */

#include "dvision/localization.hpp"
#include "dvision/parameters.hpp"

namespace dvision {

LineContainer::LineContainer(LineSegment _lineTransformed, LineType _type)
  : lineTransformed(_lineTransformed)
  , type(_type)
{
}

FeatureContainer::FeatureContainer(cv::Point2f _position, std::string _type)
  : position(_position)
  , type(_type)
{
}

Localization::Localization()
  : location(-142, 0)
  , locationKalman(location)
  , kalmanI(locationKalman)
  , _cameraProjections(NULL)
  , lastOdomX(0)
  , lastOdomY(0)
  , globalPos(0, 0, 0)
  , lastOdom(0, 0, 0)
  , lastOdomID(0)
  , ballPos(0, 0)
  , CurrentVertexId(0)
  , PreviousVertexId(0)
  , LandmarkCount(0)
  , odomLastGet(0, 0)
  , atLeastOneObservation(false)
  , nodeCounter(0)
{
    A = parameters.field_model.field_length;
    B = parameters.field_model.field_width;
    D = parameters.field_model.goal_width;
    E = parameters.field_model.goal_area_length;
    F = parameters.field_model.goal_area_width;
    G = parameters.field_model.penalty_mark_distance;
    H = parameters.field_model.center_circle_diameter;
    I = parameters.field_model.border_strip_width;
    A2 = A / 2.;
    B2 = B / 2.;
    D2 = D / 2.;
    H2 = H / 2.;
    E2 = E / 2.;
    F2 = F / 2.;
    G2 = G / 2.;
    I2 = I / 2.;

    // create the linear solver
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    // create the block solver on the top of the linear solver
    blockSolver = new g2o::BlockSolverX(linearSolver);
    // create the algorithm to carry out the optimization
    optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
}

Localization::~Localization()
{
}

bool
Localization::Init()
{
}

bool
Localization::Update(Projection& projection)
{
}

bool
Localization::Calculate(std::vector<LineSegment>&,
                        const bool& circleDetected,
                        const cv::Point2f&,
                        const std::vector<cv::Point2f>&,
                        const cv::Point2d&,
                        const std::vector<cv::Point2f>&,
                        const bool& confiused,
                        std::vector<LineContainer>&,
                        std::vector<FeatureContainer>&)
{
}

// graph
void
Localization::InitG2OGraph()
{
}

void
Localization::updateVertexIdx()
{
}

bool
Localization::addObservation(const cv::Point2d& observation, const double& xFasher, const double& yFasher, const LandmarkType& type)
{
}

// getter
cv::Point2f
Localization::GetBall()
{
}

cv::Point3d
Localization::GetLocalization()
{
}

cv::Point2d
Localization::GetOdometryFromLastGet()
{
}

// setter
void
Localization::SetBall(const cv::Point2f& _in)
{
}

double
Localization::GetUpdateCoef(const double& coef, const cv::Point2f& point)
{
}

double
Localization::GetUpdateCoef(const double& coef, const LineSegment& line)
{
}

} // namespace dvision
