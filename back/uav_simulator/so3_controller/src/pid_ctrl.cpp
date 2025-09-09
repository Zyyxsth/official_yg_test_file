#include "so3_controller/pid_ctrl.h"


namespace px4_ctrl {

  PIDctrl::PIDctrl(){

  }

  PIDctrl::PIDctrl(double dt){
    
    dt_ = dt;
    p_err_prev_ = 0;
    v_prev_ = 0;
    p_int_ = 0;
    p_dif_ = 0;
    v_ref_ = 0;
  }

  PIDctrl::~PIDctrl(){

  }

  void PIDctrl::setParams(double kp, double ki, double kd, double v_max, double a_max){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    v_max_ = v_max;
    a_max_ = a_max;
  }

  void PIDctrl::setVPrev(double v_prev){
    v_prev_ = v_prev;
  }

  void PIDctrl::setPErrPrev(double p_err_prev){
    p_err_prev_ = p_err_prev;
  }

  double PIDctrl::getPErrPrev(){
    return p_err_prev_;
  }

  double PIDctrl::getVPrev(){
    return v_prev_;
  }


  double PIDctrl::velCtrl(double p_err){

    p_int_ = p_int_ + p_err * dt_;
    p_dif_ = (p_err - p_err_prev_) / dt_; // sensor
    v_ref_ = kp_ * p_err + ki_ * p_int_ + kd_ * p_dif_;

    if((v_ref_ - v_prev_) / dt_  > a_max_){
      v_ref_ = a_max_ * dt_ + v_prev_;
    }else if((v_ref_ - v_prev_) / dt_ < -a_max_){
      v_ref_ = -a_max_ * dt_ + v_prev_;
    }

    if(v_ref_ > 0){
      v_ref_ = (v_ref_ > v_max_)? v_max_: v_ref_;
    }else{
      v_ref_ = (v_ref_ < -v_max_)? -v_max_: v_ref_;
    }
    
    p_err_prev_ = p_err;
    v_prev_ = v_ref_;
    
    return v_ref_;
  }
}