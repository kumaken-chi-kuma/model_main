#ifndef ETRC22_DEVICE_IO_H_
#define ETRC22_DEVICE_IO_H_

#include "ev3api.h"

class MotorIo {
 public:
  MotorIo();
  ~MotorIo();
  void Update();
  void SetWheelsPower(int8_t power_l, int8_t power_r);
  void StopWheels(bool brake);
  void TurnLeft();

  int32_t counts_l_;
  int32_t counts_r_;
  int8_t power_l_;
  int8_t power_r_;

 private:
  void ResetCounts();
};

class SensorIo {
 public:
  SensorIo();
  ~SensorIo();
  void Update();

  bool enter_button_pressed_;
  bool left_button_pressed_;
  bool right_button_pressed_;
  rgb_raw_t color_rgb_raw_;
  int16_t ultrasonic_sensor_distance_;
};


#endif  // ETRC22_DEVICE_IO_H_
