#include "dvision/parameters.hpp"
#include <vector>
using std::vector;
using namespace cv;

namespace dvision {

#define GPARAM(x, y)                                                                                                                                                                                   \
    do {                                                                                                                                                                                               \
        if (!m_nh->getParam(x, y)) {                                                                                                                                                                   \
            ROS_FATAL("Projection get pararm error!");                                                                                                                                                 \
            throw std::runtime_error("Projection get param error!");                                                                                                                                   \
        }                                                                                                                                                                                              \
    } while (0)

void
Parameters::init(ros::NodeHandle* nh)
{
    m_nh = nh;
    update();
}

void
Parameters::update()
{
    vector<double> dist_coeff;

    // Get camera parameters
    GPARAM("/dvision/projection/fx", parameters.camera.fx);
    GPARAM("/dvision/projection/fy", parameters.camera.fy);
    GPARAM("/dvision/projection/cx", parameters.camera.cx);
    GPARAM("/dvision/projection/cy", parameters.camera.cy);
    GPARAM("/dvision/projection/dist_coeff", dist_coeff);
    GPARAM("/dvision/projection/extrinsic_para", parameters.camera.extrinsic_para);
    GPARAM("/dvision/camera/width", parameters.camera.width);
    GPARAM("/dvision/camera/height", parameters.camera.height);

    parameters.camera.cameraMatrix = (Mat_<double>(3, 3) << parameters.camera.fx, 0, parameters.camera.cx, 0, parameters.camera.fy, parameters.camera.cy, 0, 0, 1);
    parameters.camera.distCoeff = Mat_<double>(1, 14);

    for (uint32_t i = 0; i < dist_coeff.size(); ++i) {
        parameters.camera.distCoeff.at<double>(0, i) = dist_coeff[i];
    }

    parameters.camera.imageSize = Size(parameters.camera.width, parameters.camera.height);

    // Get ball detector parameters
    GPARAM("/dvision/ball_detector/enable", parameters.ball.enable);
    GPARAM("/dvision/ball_detector/label_file", parameters.ball.label_file);
    GPARAM("/dvision/ball_detector/net_cfg", parameters.ball.net_cfg);
    GPARAM("/dvision/ball_detector/weight_file", parameters.ball.weight_file);
    GPARAM("/dvision/ball_detector/low_thresh", parameters.ball.low_thresh);
    GPARAM("/dvision/ball_detector/high_thresh", parameters.ball.high_thresh);

    // Get circle detector parameters
    GPARAM("/dvision/circle_detector/enable", parameters.circle.enable);
    GPARAM("/dvision/circle_detector/minLineLen", parameters.circle.minLineLen);
    GPARAM("/dvision/circle_detector/maxLineLen", parameters.circle.maxLineLen);
    GPARAM("/dvision/circle_detector/maxDistBetween2LS", parameters.circle.maxDistBetween2LS);
    GPARAM("/dvision/circle_detector/radiusMaxCoef", parameters.circle.radiusMaxCoef);
    GPARAM("/dvision/circle_detector/radiusMinCoef", parameters.circle.radiusMinCoef);
    GPARAM("/dvision/circle_detector/confiusedDist", parameters.circle.confiusedDist);
    GPARAM("/dvision/circle_detector/minLineSegmentCount", parameters.circle.minLineSegmentCount);

    // Get field detector parameters
    GPARAM("/dvision/field_detector/enable", parameters.field.enable);
    GPARAM("/dvision/field_detector/showMask", parameters.field.showMask);
    GPARAM("/dvision/field_detector/showResult", parameters.field.showResult);
    GPARAM("/dvision/field_detector/showDebug", parameters.field.showDebug);
    GPARAM("/dvision/field_detector/h0", parameters.field.h0);
    GPARAM("/dvision/field_detector/h1", parameters.field.h1);
    GPARAM("/dvision/field_detector/s0", parameters.field.s0);
    GPARAM("/dvision/field_detector/s1", parameters.field.s1);
    GPARAM("/dvision/field_detector/v0", parameters.field.v0);
    GPARAM("/dvision/field_detector/v1", parameters.field.v1);
    GPARAM("/dvision/field_detector/active", parameters.field.active);
    GPARAM("/dvision/field_detector/erode", parameters.field.erode);
    GPARAM("/dvision/field_detector/dilate", parameters.field.dilate);
    GPARAM("/dvision/field_detector/erode2", parameters.field.erode2);
    GPARAM("/dvision/field_detector/dilate2", parameters.field.dilate2);
    GPARAM("/dvision/field_detector/maxContourCount", parameters.field.maxContourCount);
    GPARAM("/dvision/field_detector/minArea", parameters.field.minArea);
    GPARAM("/dvision/field_detector/maxDownDiffPixel", parameters.field.maxDownDiffPixel);
    GPARAM("/dvision/field_detector/approxPoly", parameters.field.approxPoly);
    GPARAM("/dvision/field_detector/maxAcceptDistance", parameters.field.maxAcceptDistance);
    GPARAM("/dvision/field_detector/minAcceptX", parameters.field.minAcceptX);

    // Get goal detector parameters
    GPARAM("/dvision/goal_detector/enable", parameters.goal.enable);
    GPARAM("/dvision/goal_detector/showMask", parameters.goal.showMask);
    GPARAM("/dvision/goal_detector/showResult", parameters.goal.showResult);
    GPARAM("/dvision/goal_detector/showAllLines", parameters.goal.showAllLines);
    GPARAM("/dvision/goal_detector/showResLine", parameters.goal.showResLine);
    GPARAM("/dvision/goal_detector/showVote", parameters.goal.showVote);
    GPARAM("/dvision/goal_detector/h0", parameters.goal.h0);
    GPARAM("/dvision/goal_detector/h1", parameters.goal.h1);
    GPARAM("/dvision/goal_detector/s0", parameters.goal.s0);
    GPARAM("/dvision/goal_detector/s1", parameters.goal.s1);
    GPARAM("/dvision/goal_detector/v0", parameters.goal.v0);
    GPARAM("/dvision/goal_detector/v1", parameters.goal.v1);
    GPARAM("/dvision/goal_detector/active", parameters.goal.active);
    GPARAM("/dvision/goal_detector/MinLineLength", parameters.goal.MinLineLength);
    GPARAM("/dvision/goal_detector/MaxOutField", parameters.goal.MaxOutField);
    GPARAM("/dvision/goal_detector/MinNearFieldUpPoint", parameters.goal.MinNearFieldUpPoint);
    GPARAM("/dvision/goal_detector/DistanceToMerge", parameters.goal.DistanceToMerge);
    GPARAM("/dvision/goal_detector/NearestDistance", parameters.goal.NearestDistance);
    GPARAM("/dvision/goal_detector/FurthestDistance", parameters.goal.FurthestDistance);
    GPARAM("/dvision/goal_detector/NearMinLen", parameters.goal.NearMinLen);
    GPARAM("/dvision/goal_detector/NearMaxLen", parameters.goal.NearMaxLen);
    GPARAM("/dvision/goal_detector/FarMinLen", parameters.goal.FarMinLen);
    GPARAM("/dvision/goal_detector/FarMaxLen", parameters.goal.FarMaxLen);
    GPARAM("/dvision/goal_detector/jumpMax", parameters.goal.jumpMax);
    GPARAM("/dvision/goal_detector/doubleVote", parameters.goal.doubleVote);
    GPARAM("/dvision/goal_detector/minDoubleLength", parameters.goal.minDoubleLength);
    GPARAM("/dvision/goal_detector/minContinuesColor", parameters.goal.minContinuesColor);
    GPARAM("/dvision/goal_detector/extInvalidPoints", parameters.goal.extInvalidPoints);
    GPARAM("/dvision/goal_detector/extTotalPoints", parameters.goal.extTotalPoints);

    // Get line detector parameters
    GPARAM("/dvision/line_detector/enable", parameters.line.enable);
    GPARAM("/dvision/line_detector/showUnmerged", parameters.line.showUnmerged);
    GPARAM("/dvision/line_detector/showMask", parameters.line.showMask);
    GPARAM("/dvision/line_detector/showResult", parameters.line.showResult);
    GPARAM("/dvision/line_detector/showAllLine", parameters.line.showAllLine);
    GPARAM("/dvision/line_detector/showVote", parameters.line.showVote);
    GPARAM("/dvision/line_detector/showCanny", parameters.line.showCanny);
    GPARAM("/dvision/line_detector/h0", parameters.line.h0);
    GPARAM("/dvision/line_detector/h1", parameters.line.h1);
    GPARAM("/dvision/line_detector/s0", parameters.line.s0);
    GPARAM("/dvision/line_detector/s1", parameters.line.s1);
    GPARAM("/dvision/line_detector/v0", parameters.line.v0);
    GPARAM("/dvision/line_detector/v1", parameters.line.v1);
    GPARAM("/dvision/line_detector/active", parameters.line.active);
    GPARAM("/dvision/line_detector/MinLineLength", parameters.line.MinLineLength);
    GPARAM("/dvision/line_detector/AngleToMerge", parameters.line.AngleToMerge);
    GPARAM("/dvision/line_detector/DistanceToMerge", parameters.line.DistanceToMerge);
    GPARAM("/dvision/line_detector/maxLineGapHough", parameters.line.maxLineGapHough);
    GPARAM("/dvision/line_detector/rhoHough", parameters.line.rhoHough);
    GPARAM("/dvision/line_detector/thetaHough", parameters.line.thetaHough);
    GPARAM("/dvision/line_detector/thresholdHough", parameters.line.thresholdHough);
    GPARAM("/dvision/line_detector/jumpMax", parameters.line.jumpMax);
    GPARAM("/dvision/line_detector/jumpMin", parameters.line.jumpMin);
    GPARAM("/dvision/line_detector/widthCheck", parameters.line.widthCheck);
    GPARAM("/dvision/line_detector/aprxDist", parameters.line.aprxDist);
    GPARAM("/dvision/line_detector/doubleVote", parameters.line.doubleVote);
    GPARAM("/dvision/line_detector/greenVote", parameters.line.greenVote);
    GPARAM("/dvision/line_detector/colorVote", parameters.line.colorVote);
    GPARAM("/dvision/line_detector/doubleVUse", parameters.line.doubleVUse);
    GPARAM("/dvision/line_detector/greenVUse", parameters.line.greenVUse);
    GPARAM("/dvision/line_detector/colorVUse", parameters.line.colorVUse);
    GPARAM("/dvision/line_detector/doubleVStart", parameters.line.doubleVStart);
    GPARAM("/dvision/line_detector/greenVStart", parameters.line.greenVStart);
    GPARAM("/dvision/line_detector/colorVStart", parameters.line.colorVStart);
    GPARAM("/dvision/line_detector/doubleVEnd", parameters.line.doubleVEnd);
    GPARAM("/dvision/line_detector/greenVEnd", parameters.line.greenVEnd);
    GPARAM("/dvision/line_detector/colorVEnd", parameters.line.colorVEnd);
    GPARAM("/dvision/line_detector/cannyThreadshold", parameters.line.cannyThreadshold);
    GPARAM("/dvision/line_detector/blurSize", parameters.line.blurSize);
    GPARAM("/dvision/line_detector/cannyaperture", parameters.line.cannyaperture);

    // Get localization detector parameters
    GPARAM("/dvision/localization/enable", parameters.loc.enable);
    GPARAM("/dvision/localization/minLineLen", parameters.loc.minLineLen);
    GPARAM("/dvision/localization/UPDATENORMAL", parameters.loc.UPDATENORMAL);
    GPARAM("/dvision/localization/UPDATESTRONG", parameters.loc.UPDATESTRONG);
    GPARAM("/dvision/localization/UPDATEWEAK", parameters.loc.UPDATEWEAK);
    GPARAM("/dvision/localization/TOTALGAIN", parameters.loc.TOTALGAIN);
    GPARAM("/dvision/localization/VerLineMinDistanceToUpdate", parameters.loc.VerLineMinDistanceToUpdate);
    GPARAM("/dvision/localization/optimizeCounter", parameters.loc.optimizeCounter);
    GPARAM("/dvision/localization/useDeadReck", parameters.loc.useDeadReck);
    GPARAM("/dvision/localization/useKalman", parameters.loc.useKalman);
    GPARAM("/dvision/localization/useGoalPointLandMark", parameters.loc.useGoalPointLandMark);
    GPARAM("/dvision/localization/forwardRobotTrackerXY", parameters.loc.forwardRobotTrackerXY);
    GPARAM("/dvision/localization/forwardRobotTrackerZ", parameters.loc.forwardRobotTrackerZ);
    GPARAM("/dvision/localization/maxDistanceBothGoal", parameters.loc.maxDistanceBothGoal);
    GPARAM("/dvision/localization/maxDistanceSingleGoal", parameters.loc.maxDistanceSingleGoal);

    // Get field parameters
    GPARAM("/dvision/field_model/field_length", parameters.field_model.field_length);
    GPARAM("/dvision/field_model/field_width", parameters.field_model.field_width);
    GPARAM("/dvision/field_model/goal_depth", parameters.field_model.goal_depth);
    GPARAM("/dvision/field_model/goal_width", parameters.field_model.goal_width);
    GPARAM("/dvision/field_model/goal_height", parameters.field_model.goal_height);
    GPARAM("/dvision/field_model/goal_area_length", parameters.field_model.goal_area_length);
    GPARAM("/dvision/field_model/goal_area_width", parameters.field_model.goal_area_width);
    GPARAM("/dvision/field_model/penalty_mark_distance", parameters.field_model.penalty_mark_distance);
    GPARAM("/dvision/field_model/center_circle_diameter", parameters.field_model.center_circle_diameter);
    GPARAM("/dvision/field_model/border_strip_width", parameters.field_model.border_strip_width);

    // Get monitor parameters
    GPARAM("/dvision/monitor/update_gui_img", parameters.monitor.update_gui_img);
    GPARAM("/dvision/monitor/update_loc_img", parameters.monitor.update_loc_img);
}

#undef GPARAM

Parameters parameters;
} // namespace dvision
