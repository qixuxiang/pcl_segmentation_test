
#include "dmotion/MotionData.hpp"

namespace MOTION {
double sampletime;
}

// void RobotPara::readOptions(
//     ) {
//   llog(INFO) << "    ( RobotPara::readOptions reading" << std::endl;

//   cm_r = config["robot.cm_r"].as<double>();
//   cm_p = config["robot.cm_p"].as<double>();
//   cm_y = config["robot.cm_y"].as<double>();
//   cm_dx = config["robot.cm_dx"].as<double>();
//   cm_dy = config["robot.cm_dy"].as<double>();
//   /*cm_k */
//   cm_dp_fk = config["robot.cm_dp_fk"].as<double>();
//   cm_dx_fk = config["robot.cm_dx_fk"].as<double>();
//   percent_fx = config["robot.percent_fx"].as<double>();
//   cm_dx_bk = config["robot.cm_dx_bk"].as<double>();
//   percent_bx = config["robot.percent_bx"].as<double>();
//   cm_dy_lk = config["robot.cm_dy_lk"].as<double>();
//   percent_ly = config["robot.percent_ly"].as<double>();
//   cm_dy_rk = config["robot.cm_dy_rk"].as<double>();
//   percent_ry = config["robot.percent_ry"].as<double>();

//   hip_distance = config["robot.hip_distance"].as<double>();
//   upper_leg = config["robot.upper_leg"].as<double>();
//   lower_leg = config["robot.lower_leg"].as<double>();
//   upper_arm = config["robot.upper_arm"].as<double>();
//   lower_arm = config["robot.lower_arm"].as<double>();
//   lbz = config["robot.lbz"].as<double>();
//   /* ankle */
//   ra_r = config["robot.ra_r"].as<double>();
//   ra_p = config["robot.ra_p"].as<double>();
//   ra_y = config["robot.ra_y"].as<double>();
//   la_r = config["robot.la_r"].as<double>();
//   la_p = config["robot.la_p"].as<double>();
//   la_y = config["robot.la_y"].as<double>();
//   /*ankle_k */
//   la_dr_lk = config["robot.la_dr_lk"].as<double>();
//   ra_dr_lk = config["robot.ra_dr_lk"].as<double>();
//   la_dr_rk = config["robot.la_dr_rk"].as<double>();
//   ra_dr_rk = config["robot.ra_dr_rk"].as<double>();
//   la_dp_fk = config["robot.la_dp_fk"].as<double>();
//   ra_dp_fk = config["robot.ra_dp_fk"].as<double>();
//   la_dp_bk = config["robot.la_dp_bk"].as<double>();
//   ra_dp_bk = config["robot.ra_dp_bk"].as<double>();
//   /*walking amend*/
//   step_x_amend = config["robot.step_x_amend"].as<double>();
//   ad_x_max = config["robot.step_x_amend"].as<double>();
//   ad_theta_max = config["robot.step_x_amend"].as<double>();
//   mid_x_max = config["robot.mid_x_max"].as<double>();
//   mid_theta_max = config["robot.mid_theta_max"].as<double>();
//   top_x_max = config["robot.top_x_max"].as<double>();
//   top_theta_max = config["robot.top_theta_max"].as<double>();
//   back_theta_amend = config["robot.back_theta_amend"].as<double>();
//   step_theta_amend = config["robot.step_theta_amend"].as<double>();
//   mid_theta_amend  = config["robot.mid_theta_amend"].as<double>();
//   top_theta_amend  = config["robot.top_theta_amend"].as<double>();

//   /*walking ability*/
//   step_theta_max = config["robot.step_theta_max"].as<double>();
//   ad_x_max = config["robot.ad_x_max"].as<double>();
//   ad_theta_max = config["robot.ad_theta_max"].as<double>();
//   mid_x_max = config["robot.mid_x_max"].as<double>();
//   mid_theta_max = config["robot.mid_theta_max"].as<double>();
//   top_x_max = config["robot.top_x_max"].as<double>();
//   top_theta_max = config["robot.top_theta_max"].as<double>();
//   back_x_max = config["robot.back_x_max"].as<double>();
//   back_theta_max = config["robot.back_theta_max"].as<double>();
//   left_y_max = config["robot.left_y_max"].as<double>();
//   right_y_max = config["robot.right_y_max"].as<double>();
//   /*foot lifting height*/
//   ah_ml_zero = config["robot.ah_ml_zero"].as<double>();
//   ah_ml_mid = config["robot.ah_ml_mid"].as<double>();
//   ah_ml_top = config["robot.ah_ml_top"].as<double>();
//   ah_fl = config["robot.ah_fl"].as<double>();
//   ah_mr_zero = config["robot.ah_mr_zero"].as<double>();
//   ah_mr_mid = config["robot.ah_mr_mid"].as<double>();
//   ah_mr_top = config["robot.ah_mr_top"].as<double>();
//   ah_fr = config["robot.ah_fr"].as<double>();
//   /*foot placement correct in y direction*/
//   ankle_distance = config["robot.ankle_distance"].as<double>();
//   Ankle_dev_l = config["robot.ankle_dev_l"].as<double>();
//   Ankle_dev_r = config["robot.ankle_dev_r"].as<double>();
//   Ankle_dev_l_tk = config["robot.ankle_dev_l_tk"].as<double>();
//   Ankle_dev_r_tk = config["robot.ankle_dev_r_tk"].as<double>();
//   /*arm*/
//   arm_crouch_p = config["robot.arm_crouch_p"].as<double>();
//   arm_crouch_theta = config["robot.arm_crouch_theta"].as<double>();
//   /*important*/
//   stepnum = config["robot.stepnum"].as<int>();
//   stand2crouch_stepnum = config["robot.stand2crouch_stepnum"].as<int>();
//   staticExit_num = config["robot.staticexit_num"].as<int>();
//   yzmp = config["robot.yzmp"].as<double>();
//   hipheight = config["robot.hipheight"].as<double>();
//   kickPercent = config["robot.kickpercent"].as<double>();

//   /*plat diff*/
//   diffH = config["robot.diffH"].as<double>();
//   diffV = config["robot.diffV"].as<double>();
//   /*other*/
//   stepK = config["robot.stepk"].as<double>();
//   oldturning = config["robot.oldturning"].as<int>();
//   other2crouch2step_height =
//       config["robot.other2crouch2step_height"].as<double>();
//   stand2crouch2step_height =
//       config["robot.stand2crouch2step_height"].as<double>();

//   other2crouch2step_cm = config["robot.other2crouch2step_cm"].as<double>();
//   stand2crouch2step_cm = config["robot.stand2crouch2step_cm"].as<double>();

//   other2crouch2step = config["robot.other2crouch2step"].as<int>();
//   stand2crouch2step = config["robot.stand2crouch2step"].as<int>();
//   getup_bool = config["robot.getup_bool"].as<bool>();
//   llog(INFO) << "    RobotPara::readOptions succeed )" << std::endl;
// }
/*avoid error*/
double RobotPara::hip_distance = 8;
double RobotPara::upper_leg = 1.8;
double RobotPara::lower_leg = 10.8;
double RobotPara::upper_arm = 13.5;
double RobotPara::lower_arm = 13.5;
double RobotPara::lbz = 0;
double RobotPara::g = 980;
double RobotPara::dt = 0.02;
// cm
double RobotPara::cm_r = 0;
double RobotPara::cm_p = 0;
double RobotPara::cm_p_step = 0;
double RobotPara::cm_y = 0;
double RobotPara::cm_dx = 0;
double RobotPara::cm_dy = 0;
// cm_k
double RobotPara::cm_dp_fk = 0;
double RobotPara::cm_dx_fk = 0;
double RobotPara::percent_fx = 0;
double RobotPara::cm_dx_bk = 0;
double RobotPara::percent_bx = 0;
double RobotPara::cm_dy_lk = 0;
double RobotPara::percent_ly = 0;
double RobotPara::cm_dy_rk = 0;
double RobotPara::percent_ry = 0;
// ankle
double RobotPara::ra_r = 0;
double RobotPara::ra_p = 0;
double RobotPara::ra_y = 0;
double RobotPara::la_r = 0;
double RobotPara::la_p = 0;
double RobotPara::la_y = 0;
// ankle_k
double RobotPara::la_dr_lk = 0;
double RobotPara::ra_dr_lk = 0;
double RobotPara::la_dr_rk = 0;
double RobotPara::ra_dr_rk = 0;
double RobotPara::la_dp_fk = 0;
double RobotPara::ra_dp_fk = 0;
double RobotPara::la_dp_bk = 0;
double RobotPara::ra_dp_bk = 0;
/*walking amend*/
double RobotPara::step_x_amend = 0;
double RobotPara::step_theta_amend = 0;
double RobotPara::back_theta_amend = 0;
double RobotPara::mid_theta_amend = 0;
double RobotPara::top_theta_amend = 0;
/*walking ability*/
double RobotPara::step_theta_max = 0;
double RobotPara::x_compensation_acc = 0;
double RobotPara::x_compensation_dec = 0;
double RobotPara::ad_x_max = 0;
double RobotPara::ad_theta_max = 0;
double RobotPara::mid_x_max = 0;
double RobotPara::mid_theta_max = 0;
double RobotPara::top_x_max = 0;
double RobotPara::top_theta_max = 0;
double RobotPara::back_x_max = 0;
double RobotPara::back_theta_max = 0;
double RobotPara::left_y_max = 0;
double RobotPara::right_y_max = 0;
/*foot lifting height*/
double RobotPara::ah_ml_zero = 0;
double RobotPara::ah_ml_mid = 0;
double RobotPara::ah_ml_top = 0;
double RobotPara::ah_fl = 0;
double RobotPara::ah_mr_zero = 0;
double RobotPara::ah_mr_mid = 0;
double RobotPara::ah_mr_top = 0;
double RobotPara::ah_fr = 0;
/*foot placement correct in y direction*/
double RobotPara::ankle_distance = 0;
double RobotPara::Ankle_dev_l = 0;
double RobotPara::Ankle_dev_r = 0;
double RobotPara::Ankle_dev_l_tk = 0;
double RobotPara::Ankle_dev_r_tk = 0;
// arm
double RobotPara::arm_crouch_p = 0;
double RobotPara::arm_crouch_theta = 0;
/*changed important*/
int RobotPara::stepnum = 26;
int RobotPara::stand2crouch_stepnum = 50;
int RobotPara::staticExit_num       = 2;
double RobotPara::yzmp = 0;
double RobotPara::hipheight = 20;
/*kick*/
double RobotPara::kickPercent = 0;
/*plat diff*/
double RobotPara::diffH = 0;
double RobotPara::diffV = 0;
/*other*/
double RobotPara::stepK = 1;
int RobotPara::oldturning = 0;
double RobotPara::other2crouch2step_height = 0;
double RobotPara::stand2crouch2step_height = 0;
double RobotPara::stand2crouch2step_cm = 1;
double RobotPara::other2crouch2step_cm = 3;
int RobotPara::other2crouch2step = 0;
int RobotPara::stand2crouch2step = 0;
bool RobotPara::getup_bool = true;
