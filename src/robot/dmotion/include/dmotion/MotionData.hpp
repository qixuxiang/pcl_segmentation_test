#pragma once

#include "fast_math.hpp"
#include <boost/program_options.hpp>
#include <cstring>
#include <ros/ros.h>

namespace {
const int MOTORNUM = 20;
const int SMOOTHMAXERROR = 50;
const int RENUM = 100;

const int IMAGE_PLAT_DELAY = 6;
const double MAX_PLAT_YAW = 120;
const double MAX_PLAT_PITCH = 70;
const double MIN_PLAT_PITCH = 0;
}

class variables_map;
namespace MOTION {
extern double sampletime;
}

enum SUPPORT_STATUS
{
    LEFT_BASED,
    RIGHT_BASED,
    DOUBLE_BASED,
    OTHER_BASED,
    SUPPORT_STATUS_NUM
};

/**
 * body angle
 */
struct angledataDebug
{
    double angleX;
    double angleY;
    double angleZ;
    long corresCycle;

    angledataDebug()
      : angleX(0)
      , angleY(0)
      , angleZ(0)
      , corresCycle(0){};
};

/**
 * motor initial pos
 **/
struct initdataDebug
{
    double initial[MOTORNUM];

    initdataDebug()
    {
        for (int i = 0; i < MOTORNUM; i++) {
            initial[i] = 0;
        }
    }
};

// body offset
class offsetDebug
{
  public:
    // double lastOffsetX;
    // double lastOffsetY;
    double offsetX;
    double offsetY;
    long corresCycle;

  public:
    offsetDebug()
      : /*lastOffsetX(0),lastOffsetY(0),*/ offsetX(0)
      , offsetY(0)
      , corresCycle(0){};
};

class RobotPara
{
    // const params
  public:
    static void update(ros::NodeHandle* nh);
    static double hip_distance;
    static double upper_leg;
    static double lower_leg;
    static double upper_arm;
    static double lower_arm;
    static double lbz;
    static double g;
    static double dt;
    // cm
    static double cm_r;
    static double cm_p;
    static double cm_p_step;
    static double cm_y;
    static double cm_dx;
    static double cm_dy;
    // cm_k
    static double cm_dp_fk;
    static double cm_dx_fk;
    static double percent_fx;
    static double cm_dx_bk;
    static double percent_bx;
    static double cm_dy_lk;
    static double percent_ly;
    static double cm_dy_rk;
    static double percent_ry;
    // ankle
    static double ra_r;
    static double ra_p;
    static double ra_y;
    static double la_r;
    static double la_p;
    static double la_y;
    // ankle_k
    static double la_dr_lk;
    static double ra_dr_lk;
    static double la_dr_rk;
    static double ra_dr_rk;
    static double la_dp_fk;
    static double ra_dp_fk;
    static double la_dp_bk;
    static double ra_dp_bk;
    /*walking amend*/
    static double step_x_amend;
    static double step_theta_amend;
    static double back_theta_amend;
    static double mid_theta_amend;
    static double top_theta_amend;
    /*walking ability*/
    static double step_theta_max;
    static double x_compensation_acc;
    static double x_compensation_dec;
    static double ad_x_max;
    static double ad_theta_max;
    static double mid_x_max;
    static double mid_theta_max;
    static double top_x_max;
    static double top_theta_max;
    static double back_x_max;
    static double back_theta_max;
    static double left_y_max;
    static double right_y_max;
    /*foot lifting height*/
    static double ah_ml_zero;
    static double ah_ml_mid;
    static double ah_ml_top;
    static double ah_fl;
    static double ah_mr_zero;
    static double ah_mr_mid;
    static double ah_mr_top;
    static double ah_fr;
    /*foot placement correct in y direction*/
    static double ankle_distance;
    static double Ankle_dev_l;
    static double Ankle_dev_r;
    static double Ankle_dev_l_tk;
    static double Ankle_dev_r_tk;
    // arm
    static double arm_crouch_p;
    static double arm_crouch_theta;
    /*changed important*/
    static int stepnum;
    static int stand2crouch_stepnum;
    static int staticExit_num;
    static double yzmp;
    static double hipheight;
    /*kick*/
    static double kickPercent;
    static int oldturning;
    /*plat diff*/
    static double diffH;
    static double diffV;
    /*other*/
    static double stepK;
    static double other2crouch2step_height;
    static double stand2crouch2step_height;
    static double stand2crouch2step_cm;
    static double other2crouch2step_cm;
    static int other2crouch2step;
    static int stand2crouch2step;
    static bool getup_bool;
};

/**
 * Robot Control
 **/
struct RobotCtrl
{
    /* x means the step length in x direction;
     * t means the turning angle
     */
    double robot_x, robot_y, robot_t;
    int supportNum;
    /* center of mass; rightankle; left ankle */
    double cm[6], ra[6], la[6], rh[6], lh[6];
    bool auto_cm[6], auto_ra[6], auto_la[6];
    double cm_v[6], ra_v[6], la_v[6], rh_v[6], lh_v[6];
    /* center of mass offset */
    double cm_dxy[2], cm_dxy_v[2];
    SUPPORT_STATUS supportStatus;
    int num_left;
    int dataArray[MOTORNUM];

    RobotCtrl()
      : robot_x(0)
      , robot_y(0)
      , robot_t(0)
      , supportNum(0)
      , supportStatus(DOUBLE_BASED)
      , num_left(0)
    {
        memset(cm, 0, 6 * sizeof(double));
        memset(ra, 0, 6 * sizeof(double));
        memset(la, 0, 6 * sizeof(double));
        memset(cm_v, 0, 6 * sizeof(double));
        memset(ra_v, 0, 6 * sizeof(double));
        memset(la_v, 0, 6 * sizeof(double));
        memset(rh, 0, 6 * sizeof(double));
        memset(lh, 0, 6 * sizeof(double));
        memset(rh_v, 0, 6 * sizeof(double));
        memset(lh_v, 0, 6 * sizeof(double));
        memset(cm_dxy, 0, 2 * sizeof(double));
        memset(cm_dxy_v, 0, 2 * sizeof(double));
        memset(dataArray, 0, MOTORNUM * sizeof(int));
        memset(auto_cm, 0, 6 * sizeof(bool));
        memset(auto_la, 0, 6 * sizeof(bool));
        memset(auto_ra, 0, 6 * sizeof(bool));
        ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp; //理解时yzmp先可当做为0
        la[1] = RobotPara::hip_distance / 2 + RobotPara::yzmp;
        cm[2] = RobotPara::upper_leg + RobotPara::lower_leg;
        rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        rh[1] = 0;
        lh[1] = 0;
    }

    RobotCtrl(double rx, double ry, double rt, SUPPORT_STATUS ss)
      : robot_x(rx)
      , robot_y(ry)
      , robot_t(rt)
      , supportStatus(ss)
    {
        memset(cm, 0, 6 * sizeof(double));
        memset(ra, 0, 6 * sizeof(double));
        memset(la, 0, 6 * sizeof(double));
        memset(cm_v, 0, 6 * sizeof(double));
        memset(ra_v, 0, 6 * sizeof(double));
        memset(la_v, 0, 6 * sizeof(double));
        memset(rh, 0, 6 * sizeof(double));
        memset(lh, 0, 6 * sizeof(double));
        memset(rh_v, 0, 6 * sizeof(double));
        memset(lh_v, 0, 6 * sizeof(double));
        memset(cm_dxy, 0, 2 * sizeof(double));
        memset(cm_dxy_v, 0, 2 * sizeof(double));
        memset(dataArray, 0, MOTORNUM * sizeof(int));
        memset(auto_cm, 0, 6 * sizeof(bool));
        memset(auto_la, 0, 6 * sizeof(bool));
        memset(auto_ra, 0, 6 * sizeof(bool));
        ra[1] = -RobotPara::hip_distance / 2 - RobotPara::yzmp;
        la[1] = RobotPara::hip_distance / 2 + RobotPara::yzmp;
        cm[2] = RobotPara::upper_leg + RobotPara::lower_leg;
        rh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        lh[0] = RobotPara::upper_arm + RobotPara::lower_arm;
        rh[1] = 0;
        lh[1] = 0;
    }

    RobotCtrl(const RobotCtrl& robotCtrl)
    {
        robot_x = robotCtrl.robot_x;
        robot_y = robotCtrl.robot_y;
        robot_t = robotCtrl.robot_t;
        supportNum = robotCtrl.supportNum;
        for (int i = 0; i < 6; i++) {
            cm[i] = robotCtrl.cm[i];
            ra[i] = robotCtrl.ra[i];
            la[i] = robotCtrl.la[i];
            cm_v[i] = robotCtrl.cm_v[i];
            ra_v[i] = robotCtrl.ra_v[i];
            la_v[i] = robotCtrl.la_v[i];
            rh[i] = robotCtrl.rh[i];
            lh[i] = robotCtrl.lh[i];
            rh_v[i] = robotCtrl.rh_v[i];
            lh_v[i] = robotCtrl.lh_v[i];
        }
        for (int i = 0; i < 2; i++) {
            cm_dxy[i] = robotCtrl.cm_dxy[i];
            cm_dxy_v[i] = robotCtrl.cm_dxy_v[i];
        }
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = robotCtrl.dataArray[i];
        num_left = robotCtrl.num_left;
        supportStatus = robotCtrl.supportStatus;
    }

    void operator=(const RobotCtrl& robotCtrl)
    {
        robot_x = robotCtrl.robot_x;
        robot_y = robotCtrl.robot_y;
        robot_t = robotCtrl.robot_t;
        supportNum = robotCtrl.supportNum;
        for (int i = 0; i < 6; i++) {
            cm[i] = robotCtrl.cm[i];
            ra[i] = robotCtrl.ra[i];
            la[i] = robotCtrl.la[i];
            cm_v[i] = robotCtrl.cm_v[i];
            ra_v[i] = robotCtrl.ra_v[i];
            la_v[i] = robotCtrl.la_v[i];
            rh[i] = robotCtrl.rh[i];
            lh[i] = robotCtrl.lh[i];
            rh_v[i] = robotCtrl.rh_v[i];
            lh_v[i] = robotCtrl.lh_v[i];
        }
        for (int i = 0; i < 2; i++) {
            cm_dxy[i] = robotCtrl.cm_dxy[i];
            cm_dxy_v[i] = robotCtrl.cm_dxy_v[i];
        }
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = robotCtrl.dataArray[i];
        num_left = robotCtrl.num_left;
        supportStatus = robotCtrl.supportStatus;
    }

    bool operator==(const RobotCtrl& robotCtrl)
    {
        if (robot_x == robotCtrl.robot_x && robot_y == robotCtrl.robot_y && robot_t == robotCtrl.robot_t && supportStatus == robotCtrl.supportStatus && supportNum == robotCtrl.supportNum) {
            return true;
        } else {
            return false;
        }
    }

    /*Oned spline control*/
    void setAutoMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
    }

    /*walk mode cmx,cmy,la[2],ra[2] manual set*/
    void setWalkMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        auto_cm[0] = 0;
        auto_cm[1] = 0;
        auto_la[2] = 0;
        auto_ra[2] = 0;
    }

    void setKickMode()
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        auto_la[2] = 0; // z
        auto_la[0] = 0; // x
    }

    void setKickMode(SUPPORT_STATUS spt)
    {
        for (int i = 0; i < 6; i++) {
            auto_cm[i] = 1;
            auto_la[i] = 1;
            auto_ra[i] = 1;
        }
        if (spt == RIGHT_BASED) // leftkick
            auto_ra[2] = 0;
        if (spt == LEFT_BASED) // rightkick
            auto_la[2] = 0;
    }

    /* return y direction speed*/
    double getWalkVelY()
    { // not use , replaced by robot_y
        if (supportStatus == LEFT_BASED)
            return robot_y;
        else if (supportStatus == RIGHT_BASED)
            return -robot_y;
        else
            return 0;
    }

    double getComRoll(int _stepnum) // bug feature in z_stepnum
    {
        if (supportStatus == LEFT_BASED)
            return 10.0 * sin((_stepnum - num_left) / _stepnum * M_PI);
        else if (supportStatus == RIGHT_BASED)
            return -10.0 * sin((_stepnum - num_left) / _stepnum * M_PI);
        else
            return 0;
    }

    void setMotionData(int* motiondata)
    {
        for (int i = 0; i < MOTORNUM; i++)
            dataArray[i] = motiondata[i];
    }
};
