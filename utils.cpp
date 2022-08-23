#include "utils.h"

#include "app.h"

PidControl::PidControl()
    : kp_(0), ki_(0), kd_(0), dt_(EXEC_ACTION_DT_MS/1000.0) {
}

void PidControl::SetGain(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

float PidControl::CalcMv(float target_val, float current_val) {
  static float error[2];
  static float integral; 
  float p, i, d;
  static float curr_error[2];
  curr_error[0] = curr_error[1];
  curr_error[1] = current_val;
  error[0] = error[1];
  error[1] = target_val - current_val;
  integral += (error[1] + error[0])/2.0*dt_;
  p = kp_ * error[0];
  i = ki_ * integral;
  d = kd_ * (error[1]-error[0])/dt_;
  float mv = p + i + d;

  return mv;
}

LowPass::LowPass(MotorIo* motor_io) : motor_io_(motor_io){
}

void LowPass::CountsLowPass(){
  counts_l_ = motor_io_ -> counts_l_;
  counts_r_ = motor_io_ -> counts_r_;

  x_k_l[0][0] = x_kn_l[0][0];
  x_k_l[1][0] = x_kn_l[1][0];
  x_k_r[0][0] = x_kn_r[0][0];
  x_k_r[1][0] = x_kn_r[1][0];
  x_kn_l[0][0] = Ad[0][0] * x_k_l[0][0] + Ad[0][1] * x_k_l[1][0] + Bd[0][0] * counts_l_;
  x_kn_r[0][0] = Ad[0][0] * x_k_r[0][0] + Ad[0][1] * x_k_r[1][0] + Bd[0][0] * counts_r_;
  x_kn_l[0][1] = Ad[1][0] * x_k_l[0][0] + Ad[1][1] * x_k_l[1][0] + Bd[1][0] * counts_l_;
  x_kn_r[0][1] = Ad[1][0] * x_k_r[0][0] + Ad[1][1] * x_k_r[1][0] + Bd[1][0] * counts_r_;
  counts_lowpassed_l_ = Cd[0] * x_k_l[0][0] + Cd[1] * x_k_l[1][0] + Dd * counts_l_;
  counts_lowpassed_r_ = Cd[0] * x_k_r[0][0] + Cd[1] * x_k_r[1][0] + Dd * counts_r_;
}


