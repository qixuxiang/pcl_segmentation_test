#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace dvision {

struct CameraParameters
{
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeff;

    cv::Size undistImageSize;
    cv::Mat undistCameraMatrix;
    cv::Mat undistDistCoeff;

    double fx;
    double fy;
    double cx;
    double cy;

    double undistCx;
    double undistCy;

    int width;
    int height;

    int undistWidth;
    int undistHeight;

    // 16 extrinsic parameters, see meaning in Matlab code: main.m
    std::vector<double> extrinsic_para;
};

struct BallDetectorParameters
{
    bool enable;
    std::string label_file;
    std::string net_cfg;
    std::string weight_file;
    float low_thresh;
    float high_thresh;
};

struct CircleDetectorParameters
{
    bool enable;
    float minLineLen;
    float maxLineLen;
    float maxDistBetween2LS;
    float radiusMaxCoef;
    float radiusMinCoef;
    int confiusedDist;
    int minLineSegmentCount;
};

struct FieldDetectorParameters
{
    bool enable;
    bool showMask;
    bool showResult;
    bool showDebug;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    bool active;
    int erode;
    int dilate;
    int erode2;
    int dilate2;
    int maxContourCount;
    int minArea;
    int maxDownDiffPixel;
    float approxPoly;
    float maxAcceptDistance;
    float minAcceptX;
};

struct GoalDetectorParameters
{
    bool enable;
    bool showMask;
    bool showResult;
    bool showAllLines;
    bool showResLine;
    bool showVote;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    bool active;
    int MinLineLength;
    int MaxOutField;
    int MinNearFieldUpPoint;
    float DistanceToMerge;
    float NearestDistance;
    float FurthestDistance;
    int NearMinLen;
    int NearMaxLen;
    int FarMinLen;
    int FarMaxLen;
    int jumpMax;
    int doubleVote;
    int minDoubleLength;
    int minContinuesColor;
    int extInvalidPoints;
    int extTotalPoints;
};

struct LineDetectorParameters
{
    bool enable;
    bool showUnmerged;
    bool showMask;
    bool showResult;
    bool showAllLine;
    bool showVote;
    bool showCanny;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
    bool active;
    int MinLineLength;
    int AngleToMerge;
    int DistanceToMerge;
    int maxLineGapHough;
    float rhoHough;
    int thetaHough;
    int thresholdHough;
    int jumpMax;
    int jumpMin;
    float widthCheck;
    bool aprxDist;
    int doubleVote;
    int greenVote;
    int colorVote;
    bool doubleVUse;
    bool greenVUse;
    bool colorVUse;
    float doubleVStart;
    float greenVStart;
    float colorVStart;
    float doubleVEnd;
    float greenVEnd;
    float colorVEnd;
    int cannyThreadshold;
    int blurSize;
    int cannyaperture;
};

struct LocalizationParameters
{
    bool enable;
    float minLineLen;
    float UPDATENORMAL;
    float UPDATESTRONG;
    float UPDATEWEAK;
    float TOTALGAIN;
    float VerLineMinDistanceToUpdate;
    int optimizeCounter;
    bool useDeadReck;
    bool useKalman;
    bool useGoalPointLandMark;
    bool forwardRobotTrackerXY;
    bool forwardRobotTrackerZ;
    int maxDistanceBothGoal;
    int maxDistanceSingleGoal;
    int minFieldArea;
    int minDisFromCenter;
};

struct FieldModelParameters
{
    int field_length;
    int field_width;
    int goal_depth;
    int goal_width;
    int goal_height;
    int goal_area_length;
    int goal_area_width;
    int penalty_mark_distance;
    int center_circle_diameter;
    int border_strip_width;
};

struct MonitorParameters
{
    bool update_gui_img;
    bool update_loc_img;
};

struct HSVRange
{
    bool active;
    int h0;
    int h1;
    int s0;
    int s1;
    int v0;
    int v1;
};

struct Parameters
{
    BallDetectorParameters ball;
    CircleDetectorParameters circle;
    FieldDetectorParameters field;
    GoalDetectorParameters goal;
    LineDetectorParameters line;

    LocalizationParameters loc;
    CameraParameters camera;
    FieldModelParameters field_model;
    MonitorParameters monitor;

    std::string udpBroadcastAddress;
    int robotId;
    bool simulation;

    void init(ros::NodeHandle* nh);
    void update();

  private:
    ros::NodeHandle* m_nh;
};


extern Parameters parameters;
}
