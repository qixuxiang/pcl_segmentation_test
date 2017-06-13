#pragma once
#include "dmotion/GaitStateSupportLib/HumanRobotInterface.hpp"
#include "dmotion/MotionData.hpp"
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

typedef boost::numeric::ublas::matrix<double> boost_matrix;

using namespace std;

/* to do, just for runwalk to specify runkick state, to be fixed */
class GaitStateManager;
class RobotStatus;
class transitHub;

class HumanRobot : public I_HumanRobot
{
  public:
    HumanRobot(ros::NodeHandle* m_nh, transitHub* port, RobotStatus* rs, GaitStateManager* manager);
    ~HumanRobot();

    /*walk*/
    void doTxTask(int* motionData);
    void runWalk(double tsx, double tsy, double tst);
    double* curveCreate(double start, double mid, double last, int num);
    void getAngle_serial(const RobotCtrl targetCtrl, int dataArray[], const bool isExcute);
    double* get_Angle(const double* tChest, const double* tAnkle, const double* tHand, const bool whleg);
    void getVxyf0(const double tsx, double vxy[]);
    /*static*/
    int loadGaitFile(const std::string gaitname, double** data);
    void dofirstStep();
    void doCrouchFromStand(const int stepnum_);
    void doCrouchFromStandMotor(const int stepnum_);
    void doStandFromCrouch(const int stepnum_);
    void staticEntry();
    void staticExit();

  private:
    int firstStep_method;
    double* firstStep_data[5];
    int firstStep_length;

  private:
    void readOptions();
    void Leg_up(double rsx, double rsy, double rst);
    void data2q(const int data[], double lq[], double rq[]); // lq[6],rq[6],data[motornum]
    void q2data(int data[], const double lq[], const double rq[]);
    double getThetaAmend(const double tsx);
    double getRightAnkleMidHigh(const double tsx);
    double getLeftAnkleMidHigh(const double tsx);
    double getTheta(double v);

    boost_matrix rotx(double roll);
    boost_matrix roty(double pit);
    boost_matrix rotz(double yaw);
    boost_matrix rot2Tx(double roll, double x, double y, double z);
    boost_matrix rot2Ty(double pit, double x, double y, double z);
    boost_matrix rot2Tz(double yaw, double x, double y, double z);
    boost_matrix rpy2r(double roll, double pit, double yaw);
    boost_matrix rpy2t(double roll, double pit, double yaw, double x, double y, double z);
    boost_matrix Array2row(double* t_array, int row);

  private:
    /* for plat control */
    ros::NodeHandle* m_nh;
    transitHub* m_port;
    RobotStatus* m_status;
    GaitStateManager* m_manager;

    boost_matrix m_motor_bodyR;

    double curYaw, curPitch;
    double desYaw, desPitch;

    initdataDebug initdata_;

    double forward_k;
    double back_k;
    double left_k;
    double right_k;
    double angle_k;
};
