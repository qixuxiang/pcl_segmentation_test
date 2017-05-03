#include "MotionConfig.hpp"
#include "MotionData.hpp"
#include "misc/configclient/configclient.hpp"
#include "misc/utils/logger/logger.hpp"

template <>
MotionConfigClient* MotionConfigClient::m_instance = nullptr;

// update robotpara
void MotionConfig::update() {
  get_val(robot["cm_r"], RobotPara::cm_r);
  get_val(robot["cm_p"], RobotPara::cm_p);
  get_val(robot["cm_y"], RobotPara::cm_y);
  get_val(robot["cm_dx"], RobotPara::cm_dx);
  get_val(robot["cm_dy"], RobotPara::cm_dy);
  /*cm_k */
  get_val(robot["cm_dp_fk"], RobotPara::cm_dp_fk);
  get_val(robot["cm_dx_fk"], RobotPara::cm_dx_fk);
  get_val(robot["percent_fx"], RobotPara::percent_fx);
  get_val(robot["cm_dx_bk"], RobotPara::cm_dx_bk);
  get_val(robot["percent_bx"], RobotPara::percent_bx);
  get_val(robot["cm_dy_lk"], RobotPara::cm_dy_lk);
  get_val(robot["percent_ly"], RobotPara::percent_ly);
  get_val(robot["cm_dy_rk"], RobotPara::cm_dy_rk);
  get_val(robot["percent_ry"], RobotPara::percent_ry);

  get_val(robot["hip_distance"], RobotPara::hip_distance);
  get_val(robot["upper_leg"], RobotPara::upper_leg);
  get_val(robot["lower_leg"], RobotPara::lower_leg);
  get_val(robot["upper_arm"], RobotPara::upper_arm);
  get_val(robot["lower_arm"], RobotPara::lower_arm);
  get_val(robot["lbz"], RobotPara::lbz);
  /* ankle */
  get_val(robot["ra_r"], RobotPara::ra_r);
  get_val(robot["ra_p"], RobotPara::ra_p);
  get_val(robot["ra_y"], RobotPara::ra_y);
  get_val(robot["la_r"], RobotPara::la_r);
  get_val(robot["la_p"], RobotPara::la_p);
  get_val(robot["la_y"], RobotPara::la_y);
  /*ankle_k */
  get_val(robot["la_dr_lk"], RobotPara::la_dr_lk);
  get_val(robot["ra_dr_lk"], RobotPara::ra_dr_lk);
  get_val(robot["la_dr_rk"], RobotPara::la_dr_rk);
  get_val(robot["ra_dr_rk"], RobotPara::ra_dr_rk);
  get_val(robot["la_dp_fk"], RobotPara::la_dp_fk);
  get_val(robot["ra_dp_fk"], RobotPara::ra_dp_fk);
  get_val(robot["la_dp_bk"], RobotPara::la_dp_bk);
  get_val(robot["ra_dp_bk"], RobotPara::ra_dp_bk);
  /*walking amend*/
  get_val(robot["step_x_amend"], RobotPara::step_x_amend);
  get_val(robot["step_x_amend"], RobotPara::ad_x_max);
  get_val(robot["step_x_amend"], RobotPara::ad_theta_max);
  get_val(robot["mid_x_max"], RobotPara::mid_x_max);
  get_val(robot["mid_theta_max"], RobotPara::mid_theta_max);
  get_val(robot["top_x_max"], RobotPara::top_x_max);
  get_val(robot["top_theta_max"], RobotPara::top_theta_max);
  get_val(robot["back_theta_amend"], RobotPara::back_theta_amend);
  get_val(robot["step_theta_amend"], RobotPara::step_theta_amend);
  get_val(robot["mid_theta_amend"], RobotPara::mid_theta_amend);
  get_val(robot["top_theta_amend"], RobotPara::top_theta_amend);

  /*walking ability*/
  get_val(robot["step_theta_max"], RobotPara::step_theta_max);
  get_val(robot["ad_x_max"], RobotPara::ad_x_max);
  get_val(robot["ad_theta_max"], RobotPara::ad_theta_max);
  get_val(robot["mid_x_max"], RobotPara::mid_x_max);
  get_val(robot["mid_theta_max"], RobotPara::mid_theta_max);
  get_val(robot["top_x_max"], RobotPara::top_x_max);
  get_val(robot["top_theta_max"], RobotPara::top_theta_max);
  get_val(robot["back_x_max"], RobotPara::back_x_max);
  get_val(robot["back_theta_max"], RobotPara::back_theta_max);
  get_val(robot["left_y_max"], RobotPara::left_y_max);
  get_val(robot["right_y_max"], RobotPara::right_y_max);
  /*foot lifting height*/
  get_val(robot["ah_ml_zero"], RobotPara::ah_ml_zero);
  get_val(robot["ah_ml_mid"], RobotPara::ah_ml_mid);
  get_val(robot["ah_ml_top"], RobotPara::ah_ml_top);
  get_val(robot["ah_fl"], RobotPara::ah_fl);
  get_val(robot["ah_mr_zero"], RobotPara::ah_mr_zero);
  get_val(robot["ah_mr_mid"], RobotPara::ah_mr_mid);
  get_val(robot["ah_mr_top"], RobotPara::ah_mr_top);
  get_val(robot["ah_fr"], RobotPara::ah_fr);
  /*foot placement correct in y direction*/
  get_val(robot["ankle_distance"], RobotPara::ankle_distance);
  get_val(robot["ankle_dev_l"], RobotPara::Ankle_dev_l);
  get_val(robot["ankle_dev_r"], RobotPara::Ankle_dev_r);
  get_val(robot["ankle_dev_l_tk"], RobotPara::Ankle_dev_l_tk);
  get_val(robot["ankle_dev_r_tk"], RobotPara::Ankle_dev_r_tk);
  /*arm*/
  get_val(robot["arm_crouch_p"], RobotPara::arm_crouch_p);
  get_val(robot["arm_crouch_theta"], RobotPara::arm_crouch_theta);
  /*important*/
  get_val(robot["stepnum"], RobotPara::stepnum);
  get_val(robot["stand2crouch_stepnum"], RobotPara::stand2crouch_stepnum);
  get_val(robot["staticexit_num"], RobotPara::staticExit_num);
  get_val(robot["yzmp"], RobotPara::yzmp);
  get_val(robot["hipheight"], RobotPara::hipheight);
  get_val(robot["kickpercent"], RobotPara::kickPercent);

  /*plat diff*/
  get_val(robot["diffh"], RobotPara::diffH);
  get_val(robot["diffv"], RobotPara::diffV);
  /*other*/
  get_val(robot["stepk"], RobotPara::stepK);
  get_val(robot["oldturning"], RobotPara::oldturning);
  get_val(robot["other2crouch2step_height"],
          RobotPara::other2crouch2step_height);
  get_val(robot["stand2crouch2step_height"],
          RobotPara::stand2crouch2step_height);

  get_val(robot["other2crouch2step_cm"], RobotPara::other2crouch2step_cm);
  get_val(robot["stand2crouch2step_cm"], RobotPara::stand2crouch2step_cm);

  get_val(robot["other2crouch2step"], RobotPara::other2crouch2step);
  get_val(robot["stand2crouch2step"], RobotPara::stand2crouch2step);
  get_val(robot["getup_bool"], RobotPara::getup_bool);
}
