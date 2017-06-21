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
  , last_motion_info_time_(0)
  , last_delta_data_(0, 0, 0)
  , camera_projections_(NULL)
  , current_vertex_id_(0)
  , previous_vertex_id_(0)
  , landmark_count_(0)
  , at_least_one_observation_(false)
  , node_counter_(0)
{
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
    // read field model
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
    InitG2OGraph();
    optimizer.setAlgorithm(opt_algo_);
    optimizer.setVerbose(false);
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

    if (parameters.loc.useKalman) {
        kalmanI_.GetPrediction();
        cv::Point2d kal_res = kalmanI_.Update(cv::Point2d(location_.x, location_.y));
        location_kalman_.x = kal_res.x;
        location_kalman_.y = kal_res.y;
    }

    {
        boundry_n(location_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_.y, -B2_ - I_, B2_ + I_);
        boundry_n(location_kalman_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_kalman_.y, -B2_ - I_, B2_ + I_);
        // location_.x = boundry_n(location_.x, -A2_ - I_, A2_ + I_);
        // location_.y = boundry_n(location_.y, -B2_ - I_, B2_ + I_);
        // location_kalman_.x = boundry_n(location_kalman_.x, -A2_ - I_, A2_ + I_);
        // location_kalman_.y = boundry_n(location_kalman_.y, -B2_ - I_, B2_ + I_);
    }

    return true;
}

bool
Localization::Calculate(std::vector<LineSegment>& clustered_lines,
                        const bool& circle_detected,
                        const cv::Point2f& field_hull_real_center,
                        const std::vector<cv::Point2f>& field_hull_real,
                        std::vector<cv::Point2f>& field_hull_real_rotated,
                        const cv::Point2d& result_circle,
                        const std::vector<cv::Point2f>& goal_position,
                        // std::vector<LineContainer>& all_lines,
                        // std::vector<FeatureContainer>& all_features,
                        cv::Mat& m_loc_img,
                        Projection& m_projection)
{
    if (camera_projections_ == NULL || !parameters.loc.enable) {
        // ROS_ERROR("Error in programming");
        return false;
    }

    // Rotate everthing!
    std::vector<LineSegment> clustered_lines_rotated;
    cv::Point2d result_circle_rotated;
    std::vector<cv::Point2f> goal_position_real_rotated;

    m_projection.CalcHeadingOffset(clustered_lines, circle_detected, result_circle, goal_position);
    field_hull_real_rotated = m_projection.RotateTowardHeading(field_hull_real);
    clustered_lines_rotated = m_projection.RotateTowardHeading(clustered_lines);
    goal_position_real_rotated = m_projection.RotateTowardHeading(goal_position);
    result_circle_rotated = m_projection.RotateTowardHeading(result_circle);

    // Flip y from robot coord to field coord
    cv::Point2d circle_rotated_and_fliiped = result_circle_rotated;
    // circle_rotated_and_fliiped.y *= -1;

    std::vector<LineContainer> all_lines;
    std::vector<FeatureContainer> all_features;
    all_lines.reserve(clustered_lines_rotated.size());
    all_features.reserve(5);

    const double MAX_FASHER = 200.;
    at_least_one_observation_ = false;

    double UPDATENORMAL = parameters.loc.UPDATENORMAL * parameters.loc.TOTALGAIN;
    double UPDATESTRONG = parameters.loc.UPDATESTRONG * parameters.loc.TOTALGAIN;
    //double UPDATEWEAK = parameters.loc.UPDATEWEAK * parameters.loc.TOTALGAIN;

    LineSegment HorLine(cv::Point(0, -10), cv::Point(0, 10));
    LineSegment VerLine(cv::Point(10, 0), cv::Point(-10, 0));

    for (size_t i = 0; i < clustered_lines_rotated.size(); i++) {
        LineSegment line_seg = clustered_lines_rotated[i];
        // flip y coord from robot coord to field coord
        // line_seg.P1.y *= -1;
        // line_seg.P2.y *= -1;
        // 线片段长度大于预设值
        if (line_seg.GetLength() > parameters.loc.minLineLen) {
            // 取中点mid
            cv::Point2d mid = line_seg.GetMiddle();
            // 线片段与VerLine夹角小于45度
            if (line_seg.GetAbsMinAngleDegree(VerLine) < 30) {
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
                        // estimateY是机器人自身的y的估计值
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
                    // 若线片段距离检测到的场地凸包中心距离大于75 cm
                    // 并且检测到的场地凸包面积大于4 m^2
                } else if (line_seg.DistanceFromLine(field_hull_real_center) > parameters.loc.minDisFromCenter && cv::contourArea(field_hull_real_rotated) > parameters.loc.minFieldArea) {
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
                        ROS_WARN("NearVerLine!");
                        ROS_WARN("dis from hull center: %f", line_seg.DistanceFromLine(field_hull_real_center));
                        ROS_WARN("hull area: %f", cv::contourArea(field_hull_real_rotated));
                        // 同理
                    } else {
                        this_type = VerRightNear;
                        estimated_y = -B2_ + abs(mid.y);
                        ltype = RightL;
                    }
                    AddObservation(cv::Point2d(0, estimated_y), 0, MAX_FASHER * GetUpdateCoef(UPDATENORMAL, line_seg), ltype);
                }
                all_lines.push_back(LineContainer(line_seg, this_type));
            } else if (line_seg.GetAbsMinAngleDegree(HorLine) < 30) {
                LineType this_type = HorUndef;
                double angle_diff_hor = line_seg.GetExteriorAngleDegree(HorLine);
                if (angle_diff_hor < -90)
                    angle_diff_hor += 180;
                if (angle_diff_hor > 90)
                    angle_diff_hor += -180;

                // 检测到中心圆，且线片段到其距离小于30cm
                if (circle_detected && DistanceFromLineSegment(line_seg, circle_rotated_and_fliiped) < 30) {
                    this_type = HorCenter;
                    double estimated_x = -mid.x;
                    // 注意这边也是只加了estimateX，Y=0，信息矩阵中yFasher=0
                    AddObservation(cv::Point2d(estimated_x, 0), MAX_FASHER * UPDATENORMAL, 0, CenterL);
                }

                // 检测到大于2个球门柱点，且线片段与其距离均小于20cm
                // Flip goal y coord
                bool goal_pos_OK = false;
                if (goal_position_real_rotated.size() >= 2) {
                    LineSegment goal_line(goal_position_real_rotated[0], goal_position_real_rotated[1]);
                    goal_pos_OK = line_seg.DistanceFromLine(goal_position_real_rotated[0]) < parameters.loc.maxDistanceBothGoal &&
                                  line_seg.DistanceFromLine(goal_position_real_rotated[1]) < parameters.loc.maxDistanceBothGoal;
                    // && GetDistance(goal_position_real_rotated[0], goal_position_real_rotated[1]) > 50 && goal_line.GetAbsMinAngleDegree(HorLine) < 15;
                }
                // } else if (goal_position_real_rotated.size() == 1) {
                //     goal_pos_OK = goal_position_real_rotated.size() == 1 &&
                //                   line_seg.DistanceFromLine(cv::Point2f(goal_position_real_rotated[0].x, goal_position_real_rotated[0].y)) < parameters.loc.maxDistanceSingleGoal;
                // }
                if (goal_pos_OK) {
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
    //     // 找一条HorUndef的hI
    //     LineContainer hI = all_lines[i];
    //     if (hI.type_ != HorUndef)
    //         continue;
    //     for (size_t j = i; j < all_lines.size(); j++) {
    //         // 找一条HorUndef的hJ
    //         LineContainer hJ = all_lines[j];
    //         if (hJ.type_ != HorUndef)
    //             continue;
    //         cv::Point2d midI = hI.line_transformed_.GetMiddle();
    //         cv::Point2d midJ = hJ.line_transformed_.GetMiddle();
    //         double distanceToEachOther = dist3D_Segment_to_Segment(hI.line_transformed_, hJ.line_transformed_);
    //
    //         double midPointsLSAngleToOne = hI.line_transformed_.GetAbsMinAngleDegree(LineSegment(midI, midJ));
    //         //
    //         // 如果两个线片段之间的距离小于1 .5E（禁区宽度）且大于0.5E，且其夹角小于30度
    //         // 且中点连线与hI的夹角大于25度
    //         if (distanceToEachOther < E_ * 1.5 && distanceToEachOther > E_ * 0.5 && hI.line_transformed_.GetAbsMinAngleDegree(hJ.line_transformed_) < 30 && midPointsLSAngleToOne > 25) {
    //             bool hJ_Is_Goal_Line = hJ.line_transformed_.DistanceFromLine(cv::Point(0, 0)) > hI.line_transformed_.DistanceFromLine(cv::Point(0, 0));
    //             // 则选择hI与hJ中距离机器人较远的一条为Goal Line
    //             cv::Point2d mid = hJ_Is_Goal_Line ? midJ : midI;
    //             double estimatedX = 0;
    //             // 且其与机器人的距离必须大于120cm
    //             if ((hJ_Is_Goal_Line ? hJ.line_transformed_ : hI.line_transformed_).DistanceFromLine(cv::Point(0, 0)) > 120) {
    //                 LandmarkType typel = CenterL;
    //                 if (mid.x > 0) {
    //                     estimatedX = A2_ - mid.x;
    //                     typel = FrontL;
    //                 } else {
    //                     estimatedX = -A2_ + abs(mid.x);
    //                     typel = BackL;
    //                 }
    //                 double lowPC = GetUpdateCoef(UPDATEWEAK, hJ_Is_Goal_Line ? hJ.line_transformed_ : hI.line_transformed_);
    //                 AddObservation(cv::Point2d(estimatedX, 0), MAX_FASHER * lowPC, 0, typel);
    //             }
    //         }
    //     }
    // }

    // 再次添加中心圆，不过此处的将x与y都用到了
    if (circle_detected) {
        double estimated_x = -circle_rotated_and_fliiped.x;
        double estimated_y = -circle_rotated_and_fliiped.y;

        AddObservation(cv::Point2d(estimated_x, estimated_y), MAX_FASHER * UPDATESTRONG, MAX_FASHER * UPDATESTRONG, CenterL);
    }

    // add mid point of two detected goal point
    // use both estimated_x and estimated_y
    if (parameters.loc.useGoalPointLandMark && goal_position_real_rotated.size() == 2) {
        LineSegment goal_line(goal_position_real_rotated[0], goal_position_real_rotated[1], 1.0);
        if (GetDistance(goal_position_real_rotated[0], goal_position_real_rotated[1]) > 100 && goal_line.GetAbsMinAngleDegree(HorLine) < 15) {
            double estimated_x = -(goal_position_real_rotated[0].x + goal_position_real_rotated[1].x) / 2.0;
            double estimated_y = -(goal_position_real_rotated[0].y + goal_position_real_rotated[1].y) / 2.0;
            ROS_ERROR("add goal points (%f, %f) + (%f, %f) -> (%f, %f)",
                      goal_position_real_rotated[0].x,
                      goal_position_real_rotated[0].y,
                      goal_position_real_rotated[1].x,
                      goal_position_real_rotated[1].y,
                      estimated_x,
                      estimated_y);
            LandmarkType line_type;
            if (-estimated_x > 0) {
                line_type = FrontL;
                estimated_x += A2_;
            } else {
                line_type = BackL;
                estimated_x -= A2_;
            }
            double low_PC = GetUpdateCoef(UPDATESTRONG, goal_line);
            AddObservation(cv::Point2d(estimated_x, estimated_y), MAX_FASHER * low_PC, MAX_FASHER * low_PC, line_type);
        }
    }

    if (parameters.monitor.update_loc_img) {
        double offssx = 250;
        double offssy = 250;
        double ratioo = 0.2;
        // White lines
        for (uint32_t j = 0; j < all_lines.size(); ++j) {
            cv::Scalar lc;
            LineType lt = all_lines[j].type_;
            if (lt == HorUndef) {
                lc = blueMeloColor();
            } else if (lt == HorCenter) {
                lc = darkOrangeColor();
            } else if (lt == HorGoalNear || lt == HorGoal) {
                lc = pinkMeloColor();
            } else if (lt == VerUndef) {
                lc = greenColor();
            } else if (lt == VerLeft || lt == VerLeftT || lt == VerLeftNear) {
                lc = redColor();
            } else if (lt == VerRight || lt == VerRightT || lt == VerRightNear) {
                lc = yellowColor();
            } else {
                lc = whiteColor();
            }
            cv::line(m_loc_img,
                     cv::Point(all_lines[j].line_transformed_.P1.x * ratioo + offssx, -all_lines[j].line_transformed_.P1.y * ratioo + offssy),
                     cv::Point(all_lines[j].line_transformed_.P2.x * ratioo + offssx, -all_lines[j].line_transformed_.P2.y * ratioo + offssy),
                     lc,
                     2,
                     8);
        }
        // Center circle
        if (circle_detected) {
            cv::circle(m_loc_img, cv::Point(result_circle_rotated.x * ratioo + offssx, -result_circle_rotated.y * ratioo + offssy), 75 * ratioo, blueColor(), 2, 8);
        }
        // Goal
        for (size_t i = 0; i < goal_position_real_rotated.size(); i++) {
            cv::circle(m_loc_img, cv::Point(goal_position_real_rotated[i].x * ratioo + offssx, -goal_position_real_rotated[i].y * ratioo + offssy), 2, redColor(), 2, 8);
        }
        // Axis
        cv::line(m_loc_img, cv::Point(offssx, offssy), cv::Point(50 + offssx, offssy), yellowColor(), 2, 8);
        cv::line(m_loc_img, cv::Point(offssx, offssy), cv::Point(offssx, -50 + offssy), redColor(), 2, 8);
    }

    if (at_least_one_observation_) {
        UpdateVertexIdx();
        if ((node_counter_ % parameters.loc.optimizeCounter == 0) && previous_vertex_id_ > 0) {
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            Eigen::Vector3d tmpV;
            optimizer.vertex(previous_vertex_id_)->getEstimateData(tmpV.data());
            location_.x = tmpV(0);
            location_.y = tmpV(1);

            ROS_WARN("Localization: (%f, %f)", location_.x, location_.y);
            ROS_WARN("Localization_Kalman: (%f, %f)", location().x, location().y);
            return true;
        }
    }
    // ROS_WARN("robot pos: (%f, %f)", location_.x, location_.y);
    return false;
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

    if (parameters.loc.useDeadReck) {
        g2o::EdgeSE2* e = new g2o::EdgeSE2;
        e->vertices()[0] = optimizer.vertex(previous_vertex_id_);
        e->vertices()[1] = optimizer.vertex(current_vertex_id_);
        cv::Point2d dead_reck = last_delta_data();
        e->setMeasurement(g2o::SE2(dead_reck.x, dead_reck.y, 0));
        Eigen::Matrix3d information;
        information.fill(0.);
        information(0, 0) = 200;
        information(1, 1) = 200;
        information(2, 2) = 1;
        e->setInformation(information);
        optimizer.addEdge(e);
        // ROS_INFO("add Edges: %d -> dead_reck (%f, %f), info(%d, %d)", current_vertex_id_, dead_reck.x, dead_reck.y, 200, 200);
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
    // ROS_INFO("add Edges: %d -> %d (%f, %f), info(%lf, %lf)", current_vertex_id_, type, observation.x, observation.y, x_fasher, y_fasher);
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
    res.z = camera_projections_->GetHeading();
    // res.z = 0;
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

// setter
void
Localization::location(const cv::Point2d& loc)
{
    ROS_INFO("Set location to (%f, %f)", loc.x, loc.y);
    location_.x = loc.x;
    location_.y = loc.y;
    location_kalman_.x = loc.x;
    location_kalman_.y = loc.y;

    {
        boundry_n(location_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_.y, -B2_ - I_, B2_ + I_);
        boundry_n(location_kalman_.x, -A2_ - I_, A2_ + I_);
        boundry_n(location_kalman_.y, -B2_ - I_, B2_ + I_);
    }
}

cv::Point2d
Localization::last_delta_data()
{
    // Not the first iteration
    if (previous_vertex_id_ > 0) {
        cv::Point2d res(last_delta_data_.x, last_delta_data_.y);
        last_delta_data_ = cv::Point3d(0, 0, 0);
        return res;
    } else {
        return cv::Point2d(0, 0);
    }
}

double
Localization::GetUpdateCoef(const double& coef, const cv::Point2f& point)
{
    double distance = GetDistance(point);
    // ROS_WARN("DIstance: %f", distance);

    if (distance > 800.0) {
        // ROS_WARN("DIstance > 800, return 0");
        return 0;
    }
    if (distance < 300.0) {
        // ROS_WARN("DIstance < 300, return %f", coef);
        return coef;
    }
    // ROS_WARN("DIstance ~ (300, 800), return %f", coef * (1 - (distance / 800.0)));
    return coef * (1 - (distance / 800.0));
}

double
Localization::GetUpdateCoef(const double& coef, LineSegment line)
{
    double coef_updated = GetUpdateCoef(coef, line.GetClosestPointOnLineSegment(cv::Point2d(0, 0))) * line.GetProbability();
    // ROS_WARN("line prob: %f", line.GetProbability());
    // ROS_WARN("updated coef: %f", coef_updated);
    return coef_updated;
}

void
Localization::CalcDeadReckoning(const dmotion::MotionInfo& motion_info)
{
    cv::Point3d curr_delta_data(motion_info.deltaData.x, motion_info.deltaData.y, motion_info.deltaData.z);
    if (last_motion_info_time_ < motion_info.timestamp) {
        location_.x += motion_info.deltaData.x;
        location_.y += motion_info.deltaData.y;
    }
    last_delta_data_ = curr_delta_data;
    last_motion_info_time_ = motion_info.timestamp;
}

} // namespace dvision
