#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
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

    // TODO(MWX): Extrinsic parameters
};

struct CircleDetectorParameters
{
    bool enable;
    float minLineLen;
    float maxLineLen;
    float maxDistBetween2LS;
    float radiusMaxCoef;
    float radiusMinCoef;
    float confiusedDist;
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
};

struct LineDetectorParameters
{
    bool enable;
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
    float DistanceToMerge;
    int maxLineGapHough;
    float rhoHough;
    int thetaHough;
    int threasholdHough;
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
    bool useKalman;
    bool forwardRobotTrackerXY;
    bool forwardRobotTrackerZ;
};

struct Parameters
{
    CircleDetectorParameters circle;
    FieldDetectorParameters field;
    GoalDetectorParameters goal;
    LineDetectorParameters line;

    LocalizationParameters loc;
    CameraParameters camera;

    void init(ros::NodeHandle* nh);
};

extern Parameters parameters;
}
