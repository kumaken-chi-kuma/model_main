#include "device_io.h"

MotorIo::MotorIo() : counts_l_(0), counts_r_(0) {
  ev3_motor_config(EV3_PORT_A, LARGE_MOTOR);
  ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
  ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);
  ResetCounts();
}

MotorIo::~MotorIo() {
  StopWheels(false);
}

void MotorIo::Update() {
  counts_l_ = ev3_motor_get_counts(EV3_PORT_C);
  counts_r_ = ev3_motor_get_counts(EV3_PORT_B);
  power_l_ = static_cast<int8_t>(ev3_motor_get_power(EV3_PORT_C));
  power_r_ = static_cast<int8_t>(ev3_motor_get_power(EV3_PORT_B));
}

void MotorIo::SetWheelsPower(int8_t power_l, int8_t power_r) {
  const int8_t kUpperLimit = 100;
  const int8_t kLowerLimit = -100;

  if (power_l > kUpperLimit) {
    power_l = kUpperLimit;
  } else if (power_l < kLowerLimit) {
    power_l = kLowerLimit;
  }
  ev3_motor_set_power(EV3_PORT_C, power_l);

  if (power_r > kUpperLimit) {
    power_r = kUpperLimit;
  } else if (power_r < kLowerLimit) {
    power_r = kLowerLimit;
  }
  ev3_motor_set_power(EV3_PORT_B, power_r);
}

void MotorIo::StopWheels(bool brake) {
  ev3_motor_stop(EV3_PORT_B, brake);
  ev3_motor_stop(EV3_PORT_C, brake);
}

void MotorIo::ResetCounts() {
  ev3_motor_reset_counts(EV3_PORT_B);
  ev3_motor_reset_counts(EV3_PORT_C);
}

void MotorIo::TurnLeft() {
  int turn_power = 50;
  int turn_ratio = 50;
  ev3_motor_steer(EV3_PORT_B, EV3_PORT_C, turn_power, turn_ratio);
}




SensorIo::SensorIo()
    : enter_button_pressed_(false), left_button_pressed_(false), right_button_pressed_(false), color_rgb_raw_({0, 0, 0}) {
  ev3_sensor_config(EV3_PORT_1, TOUCH_SENSOR);
  ev3_sensor_config(EV3_PORT_2, COLOR_SENSOR);
  ev3_sensor_config(EV3_PORT_3, ULTRASONIC_SENSOR);
  ev3_motor_set_power(EV3_PORT_A, 20); //arm motor
}

SensorIo::~SensorIo() {
}

void SensorIo::Update() {
  ultrasonic_sensor_distance_ = ev3_ultrasonic_sensor_get_distance(EV3_PORT_3);
  enter_button_pressed_ = ev3_button_is_pressed(ENTER_BUTTON);
  left_button_pressed_ = ev3_button_is_pressed(LEFT_BUTTON);
  right_button_pressed_ = ev3_button_is_pressed(RIGHT_BUTTON);
  ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &color_rgb_raw_);
}
