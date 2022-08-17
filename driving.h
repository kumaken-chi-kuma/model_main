#ifndef ETRC22_DRIVING_H_
#define ETRC22_DRIVING_H_

#include <list>

#include "info_type.h"
#include "etrc_info.h"
#include "utils.h"

class WheelsControl {
 public:
  WheelsControl(MotorIo* motor_io);
  void Exec(int8_t target_power_l, int8_t target_power_r);

 private:
  MotorIo* motor_io_;
};

class BasicDriver {
 public:
  BasicDriver(WheelsControl* wheels_control);
  ~BasicDriver();
  void SetParam(Move move_type, int8_t base_power);
  void Run();
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Move move_type_;
  int8_t base_power_;
};

class LineTracer {
 public:
  LineTracer(WheelsControl* wheels_control, Luminous* luminous);
  ~LineTracer();
  void SetParam(Move move_type, int8_t base_power, Gain gain);
  void Run();
  void Blue_Run();//paku
  void Vgosa();//matu
  void Stop();

 private:
  WheelsControl* wheels_control_;
  Luminous* luminous_;
  Move move_type_;
  int8_t base_power_;
  const int8_t line_trace_threshold = 40;
  const int16_t line_trace_blue_threshold = 160;
  PidControl* pid_control_;
  ////matu////
  int curr_in = 0;
  float mv;
  float diff[10000]= {};
  float mv_c[10000]= {};
  uint64_t secs[100000] = {};
  uint64_t now_time = 0;
  ////matu////
};

class EndCondition {
 public:
  EndCondition(Luminous* luminous, Localize* localize);
  void SetParam(End end_type, Color end_color, float end_threshold);
  bool IsSatisfied();

 private:
  Luminous* luminous_;
  Localize* localize_;
  End end_type_;
  Color end_color_;
  float end_threshold_;
  bool end_state_;
  float ref_distance_;
  float ref_theta_;
  int count=0;//matu
};

class DrivingManager {
 public:
  DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition);
  void Update();
  void AddDrivingParam(DrivingParam param);
  bool DrivingParamsEmpty();

 private:
  void SetMoveParam(DrivingParam& param);
  void SetEndParam(DrivingParam& param);
  void Drive(DrivingParam& param);
  BasicDriver* basic_driver_;
  LineTracer* line_tracer_;
  EndCondition* end_condition_;
  std::list<DrivingParam> driving_params_;
};

#endif  // ETRC22_DRIVING_H_
