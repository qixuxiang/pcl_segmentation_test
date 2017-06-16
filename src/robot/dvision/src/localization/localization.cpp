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
#include "dvision/projection.hpp"

namespace dvision {

LineContainer::LineContainer(LineSegment lineTransformed, LineType type)
  : line_transformed_(lineTransformed)
  , type_(type)
{
}

FeatureContainer::FeatureContainer(cv::Point2f position, std::string type)
  : position_(position)
  , type_(type)
{
}

Localization::Localization()
  : location_(-142, 0)
  , location_kalman_(location_)
  , kalmanI_(location_kalman_)
  , camera_projections_(NULL)
  , last_odom_x_(0)
  , last_odom_y_(0)
  , global_pos_(0, 0, 0)
  , last_odom_(0, 0, 0)
  , last_odom_id_(0)
  , current_vertex_id_(0)
  , previous_vertex_id_(0)
  , landmark_count_(0)
  , odom_last_get_(0, 0)
  , at_least_one_observation_(false)
  , node_counter_(0)
{
    A_ = parameters.field_model.field_length;
    B_ = parameters.field_model.field_width;
    D_ = parameters.field_model.goal_width;
    E_ = parameters.field_model.goal_area_length;
    F_ = parameters.field_model.goal_area_width;
    G_ = parameters.field_model.penalty_mark_distance;
    H_ = parameters.field_model.center_circle_diameter;
    I_ = parameters.field_model.border_strip_width;
    A2_ = A_ / 2.;
    B2_ = B_ / 2.;
    D2_ = D_ / 2.;
    H2_ = H_ / 2.;
    E2_ = E_ / 2.;
    F2_ = F_ / 2.;
    G2_ = G_ / 2.;
    I2_ = I_ / 2.;

    // create the linear solver
    linear_solver_ = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
    // create the block solver on the top of the linear solver
    block_solver_ = new g2o::BlockSolverX(linear_solver_);
    // create the algorithm to carry out the optimization
    opt_algo_ = new g2o::OptimizationAlgorithmLevenberg(block_solver_);
}

Localization::~Localization()
{
    optimizer.clear();
    // TODO(corenel) GC
    // delete opt_algo_;
}

bool
Localization::Init()
{
    ROS_INFO("Localization Init() finished");
    return true;
}

bool
Localization::Update(Projection& projection)
{
    camera_projections_ = &projection;
    // if (parameters.locIsClicked) {
    //     parameters.locIsClicked = false;
    //     // ROS_INFO("Set location_ received.");
    //     location_.x = parameters.locClicked.x;
    //     location_.y = parameters.locClicked.y;
    //     location_kalman_.x = parameters.locClicked.x;
    //     location_kalman_.y = parameters.locClicked.y;
    // }

    kalmanI_.GetPrediction();
    cv::Point2d kal_res = kalmanI_.Update(cv::Point2d(location_.x, location_.y));
    location_kalman_.x = kal_res.x;
    location_kalman_.y = kal_res.y;

    {
        boundry_n(location_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_.y, -B2_ - I_, B2_ + I_);
        boundry_n(location_kalman_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_kalman_.y, -B2_ - I_, B2_ + I_);
    }

    return true;
}

bool
Localization::Calculate(std::vector<LineSegment>& clustered_lines,
                        const bool& circle_detected,
                        const cv::Point2f& field_hull_real_center,
                        const std::vector<cv::Point2f>& field_hull_real,
                        std::vector<cv::Point2f>& m_field_hull_real_rotated,
                        const cv::Point2d& result_circle,
                        const std::vector<cv::Point2f>& goal_position,
                        // std::vector<LineContainer>& all_lines,
                        // std::vector<FeatureContainer>& all_features,
                        Projection& m_projection)
{
    if (camera_projections_ == NULL || !parameters.loc.enable) {
        // ROS_ERROR("Error in programming");
        return false;
    }

    std::vector<LineContainer> all_lines;
    std::vector<FeatureContainer> all_features;

    // Rotate everthing!
    std::vector<LineSegment> clustered_lines_rotated;
    cv::Point2d result_circle_rotated;
    std::vector<cv::Point2f> goal_position_real_rotated;

    m_projection.CalcHeadingOffset(clustered_lines, circle_detected, result_circle, goal_position);
    m_field_hull_real_rotated = m_projection.RotateTowardHeading(field_hull_real);
    clustered_lines_rotated = m_projection.RotateTowardHeading(clustered_lines);
    goal_position_real_rotated = m_projection.RotateTowardHeading(goal_position);
    result_circle_rotated = m_projection.RotateTowardHeading(result_circle);

    // Flip y from robot coord to field coord
    cv::Point2d circle_rotated_and_fliiped = result_circle_rotated;
    circle_rotated_and_fliiped.y *= -1;

    all_lines.reserve(clustered_lines_rotated.size());
    all_features.reserve(5);

    const double MAX_FASHER = 200.;
    at_least_one_observation_ = false;

    double UPDATENORMAL = parameters.loc.UPDATENORMAL * parameters.loc.TOTALGAIN;
    double UPDATESTRONG = parameters.loc.UPDATESTRONG * parameters.loc.TOTALGAIN;
    double UPDATEWEAK = parameters.loc.UPDATEWEAK * parameters.loc.TOTALGAIN;

    LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
    LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

    for (size_t i = 0; i < clustered_lines_rotated.size(); i++) {
        LineSegment line_seg = clustered_lines_rotated[i];
        // flip y coord from robot coord to field coord
        line_seg.P1.y *= -1;
        line_seg.P2.y *= -1;
        // 线片段长度大于预设值
        if (line_seg.GetLength() > parameters.loc.minLineLen) {
            // 取中点mid
            cv::Point2d mid = line_seg.GetMiddle();

            // 线片段与VerLine夹角小于45度
            if (line_seg.GetAbsMinAngleDegree(VerLine) < 45) {
                // 线片段类型默认为VerUndef
                LineType this_type = VerUndef;
                // 与VerLine的夹角，取值范围为[-90, 90]
                double angle_diff_ver = line_seg.GetExteriorAngleDegree(VerLine);
                if (angle_diff_ver < -90)
                    angle_diff_ver += 180;
                if (angle_diff_ver > 90)
                    angle_diff_ver += -180;

                // 与机器人坐标系(0,0)距离大于预设值，即机器人在场地白线内的情况
                // VerLineMinDistanceToUpdate默认为70cm，即场地白线到场地边缘的距离
                if (line_seg.DistanceFromLine(cv::Point(0, 0)) > parameters.loc.VerLineMinDistanceToUpdate) {
                    // landmark类型默认为CenterL
                    LandmarkType ltype = CenterL;
                    double estimated_y = 0;
                    // 机器人在场地之内时，可根据mid.y来判断是LeftL还是RightL
                    if (mid.y > 0) {
                        this_type = VerLeft;
                        // 这个estimateY算出来的怎么感觉像是机器人自身的y的估计值
                        estimated_y = B2_ - mid.y;
                        ltype = LeftL;
                    } else {
                        this_type = VerRight;
                        estimated_y = -B2_ + abs(mid.y);
                        ltype = RightL;
                    }
                    // 给图加边，注意其估计值只有estimateY
                    // 同时在信息矩阵那边，xFasher=0
                    AddObservation(cv::Point2d(0, estimated_y), 0, MAX_FASHER * GetUpdateCoef(UPDATENORMAL, line_seg), ltype);
                    // 机器人在场地白线之外时
                    // 若线片段距离检测到的场地凸包中心距离大于70/2 cm
                    // 并且检测到的场地凸包面积大于4 m^2
                } else if (line_seg.DistanceFromLine(field_hull_real_center) > (parameters.loc.VerLineMinDistanceToUpdate / 2.) && cv::contourArea(m_field_hull_real_rotated) > 40000) {
                    LandmarkType ltype = CenterL;
                    LineSegment l2_est = line_seg;
                    if (line_seg.P1.x > line_seg.P2.x) {
                        l2_est.P1 = line_seg.P2;
                        l2_est.P2 = line_seg.P1;
                    }

                    double estimated_y = 0;
                    // 若是线在凸包中心的左边，则其为LeftL，并且是靠近机器人的VerLeftNear
                    // Determin which side of the line the 2D point is at
                    // 1 if on the right hand side;
                    // 0 if on the line;
                    // -1 if on the left hand side;
                    if (l2_est.GetSide(field_hull_real_center) < 0) {
                        this_type = VerLeftNear;
                        estimated_y = B2_ - mid.y;
                        ltype = LeftL;
                        // 同理
                    } else {
                        this_type = VerRightNear;
                        estimated_y = -B2_ + abs(mid.y);
                        ltype = RightL;
                    }
                    AddObservation(cv::Point2d(0, estimated_y), 0, MAX_FASHER * GetUpdateCoef(UPDATENORMAL, line_seg), ltype);
                }
                all_lines.push_back(LineContainer(line_seg, this_type));
            } else {
                LineType this_type = HorUndef;
                double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
                if (angle_diff_hor < -90)
                    angle_diff_hor += 180;
                if (angle_diff_hor > 90)
                    angle_diff_hor += -180;

                // 检测到中心圆，且线片段到其距离小于100cm
                if (circle_detected && DistanceFromLineSegment(line_seg, circle_rotated_and_fliiped) < 100) {
                    this_type = HorCenter;
                    double estimated_x = -mid.x;
                    // 注意这边也是只加了estimateX，Y=0，信息矩阵中yFasher=0
                    AddObservation(cv::Point2d(estimated_x, 0), MAX_FASHER * UPDATENORMAL, 0, CenterL);
                }

                // 检测到大于2个球门柱点，且线片段与其距离均小于50cm
                // Flip goal y coord
                bool double_goal_pos_OK = goal_position_real_rotated.size() >= 2 && line_seg.DistanceFromLine(cv::Point2f(goal_position_real_rotated[0].x, -goal_position_real_rotated[0].y)) < 20 &&
                                          line_seg.DistanceFromLine(cv::Point2f(goal_position_real_rotated[1].x, -goal_position_real_rotated[1].y)) < 20;
                bool single_goal_pos_OK = goal_position_real_rotated.size() == 1 && line_seg.DistanceFromLine(cv::Point2f(goal_position_real_rotated[0].x, -goal_position_real_rotated[0].y)) < 10;

                if (double_goal_pos_OK || single_goal_pos_OK) {
                    // cout << "Distance from line to goal: [0]="
                    //      << line_seg.DistanceFromLine(goal_position_real_rotated[0])
                    //      << ", [1]=" << line_seg.DistanceFromLine(goal_position_real_rotated[1]) <<
                    //      endl;
                    LandmarkType typel = CenterL;
                    double estimated_x = 0;
                    if (mid.x > 0) {
                        typel = FrontL;
                        estimated_x = A2_ - mid.x;
                    } else {
                        typel = BackL;
                        estimated_x = -A2_ + abs(mid.x);
                    }

                    double low_PC = GetUpdateCoef((goal_position_real_rotated.size() == 2 ? UPDATESTRONG : UPDATENORMAL), line_seg);
                    AddObservation(cv::Point2d(estimated_x, 0), MAX_FASHER * low_PC, 0, typel);
                    this_type = HorGoal;
                }

                all_lines.push_back(LineContainer(line_seg, this_type));
            }
        }
    }

    // 对于HorUndef的处理
    // 以下假设检测到两条HorLine，则通过条件判断其中较远的一条为球门线，较近的一条为禁区线
    // 但是现在的僵硬情况在于，球门线的后面还可能检测到一条球门后线，于是此时有三条线
    // 则此步骤中，在球门线与球门后线的判断中，球门后线就被当作了球门线，则会使得自定位x值不准
    // for (size_t i = 0; i < all_lines.size(); i++) {
    //   // 找一条HorUndef的hI
    //   LineContainer hI = all_lines[i];
    //   if (hI.type != HorUndef)
    //     continue;
    //   for (size_t j = i; j < all_lines.size(); j++) {
    //     // 找一条HorUndef的hJ
    //     LineContainer hJ = all_lines[j];
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
    //         AddObservation(cv::Point2d(estimatedX, 0), MAX_FASHER * lowPC, 0,
    //         typel);
    //       }
    //     }
    //   }
    // }

    // 再次添加中心圆，不过此处的将x与y都用到了
    if (circle_detected) {
        double estimated_x = -circle_rotated_and_fliiped.x;
        double estimated_y = -circle_rotated_and_fliiped.y;

        AddObservation(cv::Point2d(estimated_x, estimated_y), MAX_FASHER * UPDATEWEAK, MAX_FASHER * UPDATEWEAK, CenterL);
    }

    if (at_least_one_observation_) {
        UpdateVertexIdx();
        if ((node_counter_ % 30 == 0) && previous_vertex_id_ > 0) {
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            Eigen::Vector3d tmpV;
            optimizer.vertex(previous_vertex_id_)->getEstimateData(tmpV.data());
            location_.x = tmpV(0);
            location_.y = tmpV(1);

            // std::cout << "Localization: (" << location_.x << ", " << location_.y << ")" << std::endl;
            // std::cout << "-----------------------------------------------------" << std::endl;
        }
    }
    return true;
}

// graph
void
Localization::InitG2OGraph()
{
    optimizer.clear();
    landmark_count_ = 0;

    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, 0, 0));
        l->setFixed(true);
        l->setId(CenterL);
        optimizer.addVertex(l);
        landmark_count_++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(A2_, 0, 0));
        l->setFixed(true);
        l->setId(FrontL);
        optimizer.addVertex(l);
        landmark_count_++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(-A2_, 0, 0));
        l->setFixed(true);
        l->setId(BackL);
        optimizer.addVertex(l);
        landmark_count_++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, -B2_, 0));
        l->setFixed(true);
        l->setId(RightL);
        optimizer.addVertex(l);
        landmark_count_++;
    }
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(0, B2_, 0));
        l->setFixed(true);
        l->setId(LeftL);
        optimizer.addVertex(l);
        landmark_count_++;
    }
    previous_vertex_id_ = -1;
    current_vertex_id_ = landmark_count_;
    {
        g2o::VertexSE2* l = new g2o::VertexSE2;
        l->setEstimate(Eigen::Vector3d(location_.x, location_.y, 0));
        l->setFixed(false);
        l->setId(landmark_count_);
        optimizer.addVertex(l);
    }
}

void
Localization::UpdateVertexIdx()
{
    // if ((ros::Time::now() - lastSavedNodeTime).toSec() >= 0.03) {
    node_counter_++;
    // lastSavedNodeTime = ros::Time::now();
    previous_vertex_id_ = current_vertex_id_;
    current_vertex_id_++;
    if (current_vertex_id_ - landmark_count_ >= 100) {
        current_vertex_id_ = landmark_count_;
    }

    {
        g2o::VertexSE2* r = new g2o::VertexSE2;
        r->setEstimate(Eigen::Vector3d(location_.x, location_.y, 0));
        r->setFixed(false);
        r->setId(current_vertex_id_);
        if (optimizer.vertex(current_vertex_id_) != NULL) {
            optimizer.removeVertex(optimizer.vertex(current_vertex_id_));
        }

        optimizer.addVertex(r);
    }

    {
        g2o::EdgeSE2* e = new g2o::EdgeSE2;
        e->vertices()[0] = optimizer.vertex(previous_vertex_id_);
        e->vertices()[1] = optimizer.vertex(current_vertex_id_);
        cv::Point2d dead_reck = odom_last_get();
        e->setMeasurement(g2o::SE2(dead_reck.x, dead_reck.y, 0));
        Eigen::Matrix3d information;
        information.fill(0.);
        information(0, 0) = 200;
        information(1, 1) = 200;
        information(2, 2) = 1;
        e->setInformation(information);
        optimizer.addEdge(e);

        // cout << "add Edges: " << current_vertex_id_ << "-> dead_reck ("
        //      << dead_reck.x << ", " << dead_reck.y << ")" << endl;
    }
}

bool
Localization::AddObservation(cv::Point2d observation, const double& x_fasher, const double& y_fasher, const LandmarkType& type)
{
    {
        g2o::EdgeSE2* e = new g2o::EdgeSE2;

        e->vertices()[0] = optimizer.vertex(type);
        e->vertices()[1] = optimizer.vertex(current_vertex_id_);

        switch (type) {
            case RightL:
                observation.y += B2_;
                break;
            case FrontL:
                observation.x -= A2_;
                break;
            case LeftL:
                observation.y -= B2_;
                break;
            case BackL:
                observation.x += A2_;
                break;
            default:
                break;
        }
        e->setMeasurement(g2o::SE2(observation.x, observation.y, 0));
        Eigen::Matrix3d information;
        information.fill(0.);
        information(0, 0) = x_fasher;
        information(1, 1) = y_fasher;
        information(2, 2) = 1;
        e->setInformation(information);

        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        e->setRobustKernel(rk);

        optimizer.addEdge(e);
    }
    at_least_one_observation_ = true;
    ROS_DEBUG("add Edges: %d -> %d (%f, %f)", current_vertex_id_, type, observation.x, observation.y);
    return true;
}

// getter
cv::Point3d
Localization::location()
{
    cv::Point3d res;
    res.x = location_.x;
    res.y = location_.y;
    // TODO(corenel) getHeading from camera
    // res.z = camera_projections_->getHeading();
    res.z = 0;
    if (parameters.loc.useKalman) {
        res.x = location_kalman_.x;
        res.y = location_kalman_.y;
    }
    // TODO(corenel) load location_ from robot tracker
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
Localization::odom_last_get()
{
    // Not the first iteration
    if (previous_vertex_id_ > 0) {
        cv::Point2d res = odom_last_get_;
        odom_last_get_.x = odom_last_get_.y = 0;
        return res;
    } else {
        return cv::Point2d(0, 0);
    }
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
