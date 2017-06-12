#include "dmotion/GaitStateLib/GaitStateGoalie.hpp"

GaitStateGoalie::GaitStateGoalie(I_HumanRobot* robot)
  : GaitStateBase(GOALIEMID, robot)
{
}

GaitStateGoalie::~GaitStateGoalie()
{
}

// void GaitStateGoalie::readOptions(
//     const boost::program_options::variables_map& config) {
//   m_stepnum = config["robot.goalie_stepnum"].as<int>();
//   m_goalie_bool = config["robot.goalie_bool"].as<bool>();
//   m_sleeptime = config["robot.goalie_time"].as<int>();
//   m_zf_15 = config["motor.15.zf"].as<int>();
//   m_zf_17 = config["motor.17.zf"].as<int>();

// }

void
GaitStateGoalie::entry()
{
    if (m_goalie_bool) {
        doLiftbothHand();
    } else {
        doGoalieMid();
    }
    usleep(m_sleeptime * 1000000);
}

void
GaitStateGoalie::exit()
{
    robot->doCrouchFromStand(RobotPara::stepnum * 2); // crouch first
    robot->staticExit();
    // std::cout <<"stand up exit" <<std::endl;
}

void
GaitStateGoalie::execute()
{
    // robot->doCrouchFromStand(m_stepnum);
    // doRecover();
}

void
GaitStateGoalie::doLiftbothHand()
{
    RobotCtrl targetCtrl = robot->m_robotCtrl;
    // arm
    targetCtrl.lh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.rh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.lh[0] = RobotPara::upper_arm + RobotPara::lower_arm - 1;
    targetCtrl.rh[0] = RobotPara::upper_arm + RobotPara::lower_arm - 1;

    int* dataArray = new int[MOTORNUM]; // bug

    for (int i = 0; i < m_stepnum; i++) {
        dataArray[15 - 1] = robot->m_robotCtrl.num_left = m_stepnum - i;
        robot->getAngle_serial(targetCtrl, dataArray, 1);
        dataArray[15 - 1] = dataArray[15 - 1] + m_zf_15 * i * 1.0 / m_stepnum * 45.0 * 4096 / 360;
        dataArray[17 - 1] = dataArray[17 - 1] + m_zf_17 * i * 1.0 / m_stepnum * 45.0 * 4096 / 360;
        robot->doTxTask(dataArray);
    }
}

void
GaitStateGoalie::doGoalieMid()
{
    RobotCtrl targetCtrl = robot->m_robotCtrl;

    targetCtrl.cm[2] = RobotPara::hipheight - 9;

    // arm
    targetCtrl.lh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.rh[1] = RobotPara::arm_crouch_theta;
    targetCtrl.lh[0] = RobotPara::upper_arm + RobotPara::lower_arm - 0.1;
    targetCtrl.rh[0] = RobotPara::upper_arm + RobotPara::lower_arm - 0.1;

    int* dataArray = new int[MOTORNUM]; // bug

    for (int i = 0; i < m_stepnum; i++) {
        robot->m_robotCtrl.num_left = m_stepnum - i;
        robot->getAngle_serial(targetCtrl, dataArray, 1);
        dataArray[15 - 1] = dataArray[15 - 1] + m_zf_15 * i * 1.0 / m_stepnum * 20.0 * 4096 / 360;
        dataArray[17 - 1] = dataArray[17 - 1] + m_zf_17 * i * 1.0 / m_stepnum * 20.0 * 4096 / 360;
        robot->doTxTask(dataArray);
    }
}

void
GaitStateGoalie::doRecover()
{
    /* to do: crouch::getinstance()->robotctrl; */
    RobotCtrl targetCtrl = robot->m_robotCtrl;

    int* dataArray = new int[MOTORNUM]; // bug
    for (int i = 0; i < m_stepnum; i++) {
        robot->m_robotCtrl.num_left = m_stepnum - i;
        robot->getAngle_serial(targetCtrl, dataArray, 1);
        robot->doTxTask(dataArray);
    }
}
