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
    optimizer.clear();
    // TODO(corenel) GC
    // delete optimizationAlgorithm;
}

bool
Localization::Init()
{
    ROS_INFO("Localization Init() started");
    return true;
}

bool
Localization::Update(Projection& projection)
{
    _cameraProjections = &projection;
    // if (parameters.locIsClicked) {
    //     parameters.locIsClicked = false;
    //     // ROS_INFO("Set location received.");
    //     location.x = parameters.locClicked.x;
    //     location.y = parameters.locClicked.y;
    //     locationKalman.x = parameters.locClicked.x;
    //     locationKalman.y = parameters.locClicked.y;
    // }

    kalmanI.GetPrediction();
    cv::Point2d kalRes = kalmanI.Update(cv::Point2d(location.x, location.y));
    locationKalman.x = kalRes.x;
    locationKalman.y = kalRes.y;

    {
        boundry_n(location.x, -A2 - I, A2 + I);
        boundry_n(location.y, -B2 - I, B2 + I);
        boundry_n(locationKalman.x, -A2 - I, A2 + I);
        boundry_n(locationKalman.y, -B2 - I, B2 + I);
    }

    return true;
}

bool
Localization::Calculate(std::vector<LineSegment>& clusteredLines,
                        const bool& circleDetected,
                        const cv::Point2f& FieldHullRealCenter,
                        const std::vector<cv::Point2f>& FieldHullReal,
                        const cv::Point2d& resultCircleRotated,
                        const std::vector<cv::Point2f>& goalPosition,
                        std::vector<LineContainer>& AllLines,
                        std::vector<FeatureContainer>& AllFeatures)
{
    if (_cameraProjections == NULL) {
        // ROS_ERROR("Error in programming");
        return false;
    }

    // Flip y from robot coord to field coord
    cv::Point2d CircleRotatedAndFliiped = resultCircleRotated;
    CircleRotatedAndFliiped.y *= -1;

    AllLines.reserve(clusteredLines.size());
    AllFeatures.reserve(5);

    const double MAX_FASHER = 200.;
    atLeastOneObservation = false;

    double UPDATENORMAL = parameters.loc.UPDATENORMAL * parameters.loc.TOTALGAIN;
    double UPDATESTRONG = parameters.loc.UPDATESTRONG * parameters.loc.TOTALGAIN;
    double UPDATEWEAK = parameters.loc.UPDATEWEAK * parameters.loc.TOTALGAIN;

    LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
    LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

    for (size_t i = 0; i < clusteredLines.size(); i++) {
        LineSegment lineSeg = clusteredLines[i];
        // flip y coord from robot coord to field coord
        lineSeg.P1.y *= -1;
        lineSeg.P2.y *= -1;
        // 线片段长度大于预设值
        if (lineSeg.GetLength() > parameters.loc.minLineLen) {
            // 取中点mid
            cv::Point2d mid = lineSeg.GetMiddle();

            // 线片段与VerLine夹角小于45度
            if (lineSeg.GetAbsMinAngleDegree(VerLine) < 45) {
                // 线片段类型默认为VerUndef
                LineType thisType = VerUndef;
                // 与VerLine的夹角，取值范围为[-90, 90]
                double angleDiffVer = lineSeg.GetExteriorAngleDegree(VerLine);
                if (angleDiffVer < -90)
                    angleDiffVer += 180;
                if (angleDiffVer > 90)
                    angleDiffVer += -180;

                // 与机器人坐标系(0,0)距离大于预设值，即机器人在场地白线内的情况
                // VerLineMinDistanceToUpdate默认为70cm，即场地白线到场地边缘的距离
                if (lineSeg.DistanceFromLine(cv::Point(0, 0)) > parameters.loc.VerLineMinDistanceToUpdate) {
                    // landmark类型默认为CenterL
                    LandmarkType ltype = CenterL;
                    double estimatedY = 0;
                    // 机器人在场地之内时，可根据mid.y来判断是LeftL还是RightL
                    if (mid.y > 0) {
                        thisType = VerLeft;
                        // 这个estimateY算出来的怎么感觉像是机器人自身的y的估计值
                        estimatedY = B2 - mid.y;
                        ltype = LeftL;
                    } else {
                        thisType = VerRight;
                        estimatedY = -B2 + abs(mid.y);
                        ltype = RightL;
                    }
                    // 给图加边，注意其估计值只有estimateY
                    // 同时在信息矩阵那边，xFasher=0
                    addObservation(cv::Point2d(0, estimatedY), 0, MAX_FASHER * GetUpdateCoef(UPDATENORMAL, lineSeg), ltype);
                    // 机器人在场地白线之外时
                    // 若线片段距离检测到的场地凸包中心距离大于70/2 cm
                    // 并且检测到的场地凸包面积大于4 m^2
                } else if (lineSeg.DistanceFromLine(FieldHullRealCenter) > (parameters.loc.VerLineMinDistanceToUpdate / 2.) && cv::contourArea(FieldHullReal) > 40000) {
                    LandmarkType ltype = CenterL;
                    LineSegment l2Test = lineSeg;
                    if (lineSeg.P1.x > lineSeg.P2.x) {
                        l2Test.P1 = lineSeg.P2;
                        l2Test.P2 = lineSeg.P1;
                    }

                    double estimatedY = 0;
                    // 若是线在凸包中心的左边，则其为LeftL，并且是靠近机器人的VerLeftNear
                    // Determin which side of the line the 2D point is at
                    // 1 if on the right hand side;
                    // 0 if on the line;
                    // -1 if on the left hand side;
                    if (l2Test.GetSide(FieldHullRealCenter) < 0) {
                        thisType = VerLeftNear;
                        estimatedY = B2 - mid.y;
                        ltype = LeftL;
                        // 同理
                    } else {
                        thisType = VerRightNear;
                        estimatedY = -B2 + abs(mid.y);
                        ltype = RightL;
                    }
                    addObservation(cv::Point2d(0, estimatedY), 0, MAX_FASHER * GetUpdateCoef(UPDATENORMAL, lineSeg), ltype);
                }
                AllLines.push_back(LineContainer(lineSeg, thisType));
            } else {
                LineType thisType = HorUndef;
                double angleDiffHor = lineSeg.GetExteriorAngleDegree(HorLine);
                if (angleDiffHor < -90)
                    angleDiffHor += 180;
                if (angleDiffHor > 90)
                    angleDiffHor += -180;

                // 检测到中心圆，且线片段到其距离小于100cm
                if (circleDetected && DistanceFromLineSegment(lineSeg, CircleRotatedAndFliiped) < 100) {
                    thisType = HorCenter;
                    double estimatedX = -mid.x;
                    // 注意这边也是只加了estimateX，Y=0，信息矩阵中yFasher=0
                    addObservation(cv::Point2d(estimatedX, 0), MAX_FASHER * UPDATENORMAL, 0, CenterL);
                }

                // 检测到大于2个球门柱点，且线片段与其距离均小于50cm
                // Flip goal y coord
                bool doubleGoalPosOK = goalPosition.size() >= 2 && lineSeg.DistanceFromLine(cv::Point2f(goalPosition[0].x, -goalPosition[0].y)) < 20 &&
                                       lineSeg.DistanceFromLine(cv::Point2f(goalPosition[1].x, -goalPosition[1].y)) < 20;
                bool singleGoalPosOK = goalPosition.size() == 1 && lineSeg.DistanceFromLine(cv::Point2f(goalPosition[0].x, -goalPosition[0].y)) < 10;

                if (doubleGoalPosOK || singleGoalPosOK) {
                    // cout << "Distance from line to goal: [0]="
                    //      << lineSeg.DistanceFromLine(goalPosition[0])
                    //      << ", [1]=" << lineSeg.DistanceFromLine(goalPosition[1]) <<
                    //      endl;
                    LandmarkType typel = CenterL;
                    double estimatedX = 0;
                    if (mid.x > 0) {
                        typel = FrontL;
                        estimatedX = A2 - mid.x;
                    } else {
                        typel = BackL;
                        estimatedX = -A2 + abs(mid.x);
                    }

                    double lowPC = GetUpdateCoef((goalPosition.size() == 2 ? UPDATESTRONG : UPDATENORMAL), lineSeg);
                    addObservation(cv::Point2d(estimatedX, 0), MAX_FASHER * lowPC, 0, typel);
                    thisType = HorGoal;
                }

                AllLines.push_back(LineContainer(lineSeg, thisType));
            }
        }
    }

    // 对于HorUndef的处理
    // 以下假设检测到两条HorLine，则通过条件判断其中较远的一条为球门线，较近的一条为禁区线
    // 但是现在的僵硬情况在于，球门线的后面还可能检测到一条球门后线，于是此时有三条线
    // 则此步骤中，在球门线与球门后线的判断中，球门后线就被当作了球门线，则会使得自定位x值不准
    // for (size_t i = 0; i < AllLines.size(); i++) {
    //   // 找一条HorUndef的hI
    //   LineContainer hI = AllLines[i];
    //   if (hI.type != HorUndef)
    //     continue;
    //   for (size_t j = i; j < AllLines.size(); j++) {
    //     // 找一条HorUndef的hJ
    //     LineContainer hJ = AllLines[j];
    //     if (hJ.type != HorUndef)
    //       continue;
    //     cv::Point2d midI = hI.lineTransformed.GetMiddle();
    //     cv::Point2d midJ = hJ.lineTransformed.GetMiddle();
    //     double distanceToEachOther =
    //         dist3D_Segment_to_Segment(hI.lineTransformed, hJ.lineTransformed);
    //
    //     double midPointsLSAngleToOne =
    //         hI.lineTransformed.GetAbsMinAngleDegree(LineSegment(midI, midJ));
    //     //
    //     如果两个线片段之间的距离小于1.5E（禁区宽度）且大于0.5E，且其夹角小于30度
    //     // 且中点连线与hI的夹角大于25度
    //     if (distanceToEachOther < E * 1.5 && distanceToEachOther > E * 0.5 &&
    //         hI.lineTransformed.GetAbsMinAngleDegree(hJ.lineTransformed) < 30 &&
    //         midPointsLSAngleToOne > 25) {
    //       bool hJ_Is_Goal_Line =
    //           hJ.lineTransformed.DistanceFromLine(cv::Point(0, 0)) >
    //           hI.lineTransformed.DistanceFromLine(cv::Point(0, 0));
    //       // 则选择hI与hJ中距离机器人较远的一条为Goal Line
    //       cv::Point2d mid = hJ_Is_Goal_Line ? midJ : midI;
    //       double estimatedX = 0;
    //       // 且其与机器人的距离必须大于120cm
    //       if ((hJ_Is_Goal_Line ? hJ.lineTransformed : hI.lineTransformed)
    //               .DistanceFromLine(cv::Point(0, 0)) > 120) {
    //         LandmarkType typel = CenterL;
    //         if (mid.x > 0) {
    //           estimatedX = A2 - mid.x;
    //           typel = FrontL;
    //         } else {
    //           estimatedX = -A2 + abs(mid.x);
    //           typel = BackL;
    //         }
    //         double lowPC =
    //             GetUpdateCoef(UPDATESTRONG, hJ_Is_Goal_Line ?
    //             hJ.lineTransformed
    //                                                         :
    //                                                         hI.lineTransformed);
    //         addObservation(cv::Point2d(estimatedX, 0), MAX_FASHER * lowPC, 0,
    //         typel);
    //       }
    //     }
    //   }
    // }

    // 再次添加中心圆，不过此处的将x与y都用到了
    if (circleDetected) {
        double estimatedX = -CircleRotatedAndFliiped.x;
        double estimatedY = -CircleRotatedAndFliiped.y;

        addObservation(cv::Point2d(estimatedX, estimatedY), MAX_FASHER * UPDATEWEAK, MAX_FASHER * UPDATEWEAK, CenterL);
    }

    if (atLeastOneObservation) {
        updateVertexIdx();
        if ((nodeCounter % 30 == 0) && PreviousVertexId > 0) {
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            Eigen::Vector3d tmpV;
            optimizer.vertex(PreviousVertexId)->getEstimateData(tmpV.data());
            location.x = tmpV(0);
            location.y = tmpV(1);

            std::cout << "Localization: (" << location.x << ", " << location.y << ")" << std::endl;
            std::cout << "-----------------------------------------------------" << std::endl;
        }
    }
    return true;
}

// graph
void
Localization::InitG2OGraph()
{
    optimizer.clear();
    LandmarkCount = 0;

    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, 0, 0));
        l->setFixed(true);
        l->setId(CenterL);
        optimizer.addVertex(l);
        LandmarkCount++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(A2, 0, 0));
        l->setFixed(true);
        l->setId(FrontL);
        optimizer.addVertex(l);
        LandmarkCount++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(-A2, 0, 0));
        l->setFixed(true);
        l->setId(BackL);
        optimizer.addVertex(l);
        LandmarkCount++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, -B2, 0));
        l->setFixed(true);
        l->setId(RightL);
        optimizer.addVertex(l);
        LandmarkCount++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, B2, 0));
        l->setFixed(true);
        l->setId(LeftL);
        optimizer.addVertex(l);
        LandmarkCount++;
    }
    PreviousVertexId = -1;
    CurrentVertexId = LandmarkCount;
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(location.x, location.y, 0));
        l->setFixed(false);
        l->setId(LandmarkCount);
        optimizer.addVertex(l);
    }
}

void
Localization::updateVertexIdx()
{
    // if ((ros::Time::now() - lastSavedNodeTime).toSec() >= 0.03) {
    nodeCounter++;
    // lastSavedNodeTime = ros::Time::now();
    PreviousVertexId = CurrentVertexId;
    CurrentVertexId++;
    if (CurrentVertexId - LandmarkCount >= 100) {
        CurrentVertexId = LandmarkCount;
    }

    {
        g2o::VertexSE2* r = new g2o::VertexSE2;
        r->setEstimate(Eigen::Vector3d(location.x, location.y, 0));
        r->setFixed(false);
        r->setId(CurrentVertexId);
        if (optimizer.vertex(CurrentVertexId) != NULL) {
            optimizer.removeVertex(optimizer.vertex(CurrentVertexId));
        }

        optimizer.addVertex(r);
    }

    {
        g2o::EdgeSE2* e = new g2o::EdgeSE2;
        e->vertices()[0] = optimizer.vertex(PreviousVertexId);
        e->vertices()[1] = optimizer.vertex(CurrentVertexId);
        cv::Point2d dead_reck = GetOdometryFromLastGet();
        e->setMeasurement(g2o::SE2(dead_reck.x, dead_reck.y, 0));
        Eigen::Matrix3d information;
        information.fill(0.);
        information(0, 0) = 200;
        information(1, 1) = 200;
        information(2, 2) = 1;
        e->setInformation(information);
        optimizer.addEdge(e);

        // cout << "add Edges: " << CurrentVertexId << "-> dead_reck ("
        //      << dead_reck.x << ", " << dead_reck.y << ")" << endl;
    }
}

bool
Localization::addObservation(cv::Point2d observation, const double& xFasher, const double& yFasher, const LandmarkType& type)
{
    {
        g2o::EdgeSE2* e = new g2o::EdgeSE2;

        e->vertices()[0] = optimizer.vertex(type);
        e->vertices()[1] = optimizer.vertex(CurrentVertexId);

        switch (type) {
            case RightL:
                observation.y += B2;
                break;
            case FrontL:
                observation.x -= A2;
                break;
            case LeftL:
                observation.y -= B2;
                break;
            case BackL:
                observation.x += A2;
                break;
            default:
                break;
        }
        e->setMeasurement(g2o::SE2(observation.x, observation.y, 0));
        Eigen::Matrix3d information;
        information.fill(0.);
        information(0, 0) = xFasher;
        information(1, 1) = yFasher;
        information(2, 2) = 1;
        e->setInformation(information);

        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        e->setRobustKernel(rk);

        optimizer.addEdge(e);
    }
    atLeastOneObservation = true;
    // cout << "add Edges: " << CurrentVertexId << "->" << type << " ("
    //      << observation.x << ", " << observation.y << ")" << endl;
    return true;
}

// getter
cv::Point2f
Localization::GetBall()
{
    return ballPos;
}

cv::Point3d
Localization::GetLocalization()
{
    cv::Point3d res;
    res.x = location.x;
    res.y = location.y;
    // TODO(corenel) getHeading from camera
    // res.z = _cameraProjections->getHeading();
    res.z = 0;
    if (parameters.loc.useKalman) {
        res.x = locationKalman.x;
        res.y = locationKalman.y;
    }
    // TODO(corenel) load location from robot tracker
    // if (parameters.loc.forwardRobotTrackerXY) {
    //   res.x = robotTrackerLoc.x;
    //   res.y = robotTrackerLoc.y;
    // }
    // if (parameters.loc.forwardRobotTrackerZ) {
    //   res.z = robotTrackerLoc.z;
    // }

    return res;
}

cv::Point2d
Localization::GetOdometryFromLastGet()
{
    // Not the first iteration
    if (PreviousVertexId > 0) {
        cv::Point2d res = odomLastGet;
        odomLastGet.x = odomLastGet.y = 0;
        return res;
    } else {
        return cv::Point2d(0, 0);
    }
}

// setter
void
Localization::SetBall(const cv::Point2f& _in)
{
    ballPos = _in;
}

double
Localization::GetUpdateCoef(const double& coef, const cv::Point2f& point)
{
    double distance = GetDistance(point);

    if (distance > 800) {
        return 0;
    }
    if (distance < 300) {
        return coef;
    }

    return coef * (1 - (distance / 800));
}

double
Localization::GetUpdateCoef(const double& coef, LineSegment line)
{
    return GetUpdateCoef(coef, line.GetClosestPointOnLineSegment(cv::Point2d(0, 0))) * line.GetProbability();
}

} // namespace dvision
