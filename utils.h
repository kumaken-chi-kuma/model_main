#ifndef ETRC22_UTILS_H_
#define ETRC22_UTILS_H_

#include "device_io.h"

class PidControl {
 public:
  PidControl();
  void SetGain(float kp, float ki, float kd);
  float CalcMv(float target_val, float current_val);

 private:
  float kp_;
  float ki_;
  float kd_;
  float dt_;
};

class LowPass {
 public:
  LowPass(MotorIo* motor_io);
  void CountsLowPass();
  float counts_lowpassed_l_ = 0;
  float counts_lowpassed_r_ = 0;
  int counts_l_;
  int counts_r_;

 private:
  MotorIo* motor_io_;

  float Ad[2][2] = {{1.044 ,-0.5447},{0.5, 0}};
  float Bd[2][1] = {{0.5}, {0}};
  float Cd[2] = {0.3479, 0.1663};
  float Dd = 0.05715;
  float x_k_l[2][1] = {};
  float x_k_r[2][1] = {};
  float x_kn_l[2][1] = {};
  float x_kn_r[2][1] = {};
  float y_k_l = 0;
  float y_k_r = 0;


};

#endif  // ETRC22_UTILS_H_