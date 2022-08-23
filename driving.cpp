#include "driving.h"

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  int8_t curr_power_l = motor_io_->power_l_;
  if (target_power_l - curr_power_l > 20) {
    target_power_l = curr_power_l + 20;
  } else if (target_power_l - curr_power_l < -20) {
    target_power_l = curr_power_l - 20;
  }

  int8_t curr_power_r = motor_io_->power_r_;
  if (target_power_r - curr_power_r > 20) {
    target_power_r = curr_power_r + 20;
  } else if (target_power_r - curr_power_r < -20) {
    target_power_r = curr_power_r - 20;
  }
  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(target_power_l, target_power_r);
  }
}

BasicDriver::BasicDriver(WheelsControl* wheels_control, LowPass* low_pass)
    : wheels_control_(wheels_control), low_pass_(low_pass),
      move_type_(kInvalidMove), base_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void BasicDriver::Run() {
  counts_l_last = counts_l_now;
  counts_r_last = counts_r_now;
  counts_l_now = low_pass_-> counts_lowpassed_l_;
  counts_r_now = low_pass_-> counts_lowpassed_r_;

  velocity_l = (counts_l_now - counts_l_last)/delta_time;
  velocity_r = (counts_r_now - counts_r_last)/delta_time;

  if (move_type_ == kGoForward) {
    velocity_target_l = base_power_;
    velocity_target_r = base_power_;
    PidControlVelocity();
  } else if (move_type_ == kGoBackward) {
    power_l = power_r = -base_power_;
  } else if (move_type_ == kRotateLeft || move_type_ == kRotateLeftBonus) {
    velocity_target_l = base_power_;
    velocity_target_r = base_power_;
    gain_velocity_control[0][1] = 0.15;
    gain_velocity_control[1][1] = 0.15;
    PidControlVelocity();
  } else if (move_type_ == kRotateRight || move_type_ == kRotateRightBonus) {
    velocity_target_l = base_power_;
    velocity_target_r = base_power_;
    gain_velocity_control[0][1] = 0.15;
    gain_velocity_control[1][1] = 0.15;
    PidControlVelocity();
  } else if (move_type_ == kStayM){
    power_l = power_r = 0;
  } else if (move_type_ == kRotateLeft_No_R) {
    power_l = 0;
    power_r = base_power_;
  } else if (move_type_ == kRotateRight_No_L) {
    power_l = base_power_;
    power_r = 0;
  } else{
    power_l = power_r = 0;
  }

  wheels_control_->Exec(power_l, power_r);
}

void BasicDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

/////////////////////////////pid///////////////////////////////////////
void BasicDriver::PidControlVelocity(){
  error_last_l = error_l;
  error_last_r = error_r;
  error_l = velocity_target_l - velocity_l;
  error_r = velocity_target_r - velocity_r;

  error_integral_l += error_l * delta_time;
  error_integral_r += error_r * delta_time;
  error_differential_l = (error_l - error_last_l) / delta_time;
  error_differential_r = (error_r - error_last_r) / delta_time;

  power_l = (int)(gain_velocity_control[0][0] * error_l + gain_velocity_control[0][1] * error_integral_l + gain_velocity_control[0][2] * error_differential_l);
  power_r = (int)(gain_velocity_control[1][0] * error_r + gain_velocity_control[1][1] * error_integral_r + gain_velocity_control[1][2] * error_differential_r);

  if(power_l > 100){
    power_l = 100;
  }else if(power_l < -100){
    power_l = -100;
  }
  if(power_r > 100){
    power_r = 100;
  }else if(power_r < -100){
    power_r = -100;
  }
}
/////////////////////////////pid///////////////////////////////////////


/////////////////////////////////経路追従////////////////////////////////////

PursuitDriver::PursuitDriver(WheelsControl* wheels_control, PurePursuit* pure_pursuit)
    : wheels_control_(wheels_control), pure_pursuit_(pure_pursuit),
      move_type_(kInvalidMove), base_power_(0) {
}

PursuitDriver::~PursuitDriver() {
}

void PursuitDriver::SetParam(Move move_type, int base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void PursuitDriver::Run() {
  float target_distance_ = pure_pursuit_->target_distance;
  float difference_rad_ = pure_pursuit_->difference_rad;
  p_power_l = gain_kv_l * target_distance_ - gain_kt_l * difference_rad_ + base_power_;
  p_power_r = gain_kv_r * target_distance_ + gain_kt_r * difference_rad_ + base_power_;

  int power_l = (int)p_power_l;
  int power_r = (int)p_power_r;

  wheels_control_->Exec(power_l, power_r);
}

void PursuitDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

/////////////////////////////////経路追従////////////////////////////////////

LineTracer::LineTracer(WheelsControl* wheels_control, Luminous* luminous)
    : wheels_control_(wheels_control), luminous_(luminous),
      move_type_(kInvalidMove), base_power_(0) {
  pid_control_ = new PidControl();
}

LineTracer::~LineTracer() {
  delete pid_control_;
}

void LineTracer::SetParam(Move move_type, int8_t base_power, Gain gain) {
  move_type_ = move_type;
  base_power_ = base_power;
  pid_control_->SetGain(gain.kp, gain.ki, gain.kd);
}

void LineTracer::Run() {
  float curr_hsv = luminous_->hsv_.v;
  float mv = pid_control_->CalcMv(line_trace_threshold, curr_hsv);

  //////////////////////////////matu/////////////////////////////
  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }
  int8_t power_l;
  int8_t power_r;

  if (move_type_ == kRightcurve){
  power_l = static_cast<int8_t>(base_power_ + mv + 30);
  power_r = static_cast<int8_t>(base_power_ - mv - 25 );
  } else if(move_type_ == kLeftcurve){
  power_l = static_cast<int8_t>(base_power_ + mv - 15 );
  power_r = static_cast<int8_t>(base_power_ - mv + 25);
  }else {
  power_l = static_cast<int8_t>(base_power_ + mv);
  power_r = static_cast<int8_t>(base_power_ - mv);
  }
    wheels_control_->Exec(power_l, power_r);
  //////////////////////////////matu/////////////////////////////
}

void LineTracer::Blue_Run() {
  float curr_hsv = luminous_->hsv_.v;
  float mv = pid_control_->CalcMv(line_trace_blue_threshold, curr_hsv);

  if (move_type_ == kTraceBlueLeftEdge) {
    mv *= -1;
  }
  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

void LineTracer::Stop() {
  wheels_control_->Exec(0, 0);
}


EndCondition::EndCondition(Luminous* luminous, Localize* localize)
    : luminous_(luminous), localize_(localize),
      end_type_(kInvalidEnd), end_color_(kInvalidColor), end_threshold_(0),
      end_state_(false), ref_distance_(0), ref_theta_(0) {
}

void EndCondition::SetParam(End end_type, Color end_color, float end_threshold) {
  end_type_ = end_type;
  end_color_ = end_color;
  end_threshold_ = end_threshold;
  end_state_ = false;

  if (end_type_ == kDistanceEnd) {
     ref_distance_ = localize_->distance_;
  }
  else if (end_type_ == kThetaEnd) {
    ref_theta_ = localize_ -> theta_;
  }
}

bool EndCondition::IsSatisfied() {
  switch (end_type_) {
    case kColorEnd:
      if (end_color_ == luminous_->color_)
        end_state_ = true;
      break;

    case kDistanceEnd:
      if (end_threshold_ > 0 && localize_->distance_ - ref_distance_ > end_threshold_)
        end_state_ = true;
      else if (end_threshold_ < 0 && localize_->distance_ - ref_distance_ < end_threshold_)
        end_state_ = true;
      break;

    case kThetaEnd:
      if (end_threshold_ > 0 && localize_->theta_ - ref_theta_ > end_threshold_)
        end_state_ = true;
      else if (end_threshold_ < 0 && localize_->theta_ - ref_theta_ < end_threshold_)
        end_state_ = true;
      break;
    case kTimeEnd:

      break;
    default:
      break;
  }

  return end_state_;
}

DrivingManager::DrivingManager(BasicDriver* basic_driver, PursuitDriver* pursuit_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), pursuit_driver_(pursuit_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
}

void DrivingManager::Update() {
  if (driving_params_.size() <= 0) {
    return;
  }

  DrivingParam& curr_param = driving_params_.front();
  if (!curr_param.is_started) {//true
    SetMoveParam(curr_param);
    SetEndParam(curr_param);
    curr_param.is_started = true;
  }

  Drive(curr_param);

  if (end_condition_->IsSatisfied()) {
    curr_param.is_finished = true;
  }

  if (curr_param.is_finished) {
    driving_params_.pop_front();
  }

  if (driving_params_.empty()) {
    basic_driver_->Stop();
  }
}

void DrivingManager::AddDrivingParam(DrivingParam param) {
  driving_params_.push_back(param);
}

bool DrivingManager::DrivingParamsEmpty() {
  return driving_params_.empty();
}

void DrivingManager::SetMoveParam(DrivingParam& param) {
  Move move_type = param.move_type;
  int8_t base_power = param.base_power;
  Gain gain = param.gain;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
    case kRightcurve:
    case kLeftcurve:
    case kTraceBlueLeftEdge:
    case kTraceBlueRightEdge:
      line_tracer_->SetParam(move_type, base_power, gain);
      break;

    case kPursuitLeft:
    case kPursuitRight:
      pursuit_driver_->SetParam(move_type, base_power);
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
    case kRotateLeft_No_R:
    case kRotateRight_No_L:
      basic_driver_->SetParam(move_type, base_power);
      break;

    default:
      break;
  }
}

void DrivingManager::SetEndParam(DrivingParam& param) {
  End end_type = param.end_type;
  Color end_color = param.end_color;
  float end_threshold = param.end_threshold;

  end_condition_->SetParam(end_type, end_color, end_threshold);
}

void DrivingManager::Drive(DrivingParam& param) {
  Move move_type = param.move_type;

  switch (move_type) {
    case kTraceLeftEdge:
    case kTraceRightEdge:
    case kRightcurve:
    case kLeftcurve:
      line_tracer_->Run();
      break;

    case kTraceBlueLeftEdge:
    case kTraceBlueRightEdge:
      line_tracer_->Blue_Run();
      break;

    case kPursuitLeft:
    case kPursuitRight:
      pursuit_driver_->Run();
      break;

    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
    case kRotateLeft_No_R:
    case kRotateRight_No_L:
      basic_driver_->Run();
      break;

    case kStopWheels:
      basic_driver_->Stop();
      break;

    default:
      break;
  }
}
