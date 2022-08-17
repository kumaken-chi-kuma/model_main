#include "driving.h"

WheelsControl::WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void WheelsControl::Exec(int8_t target_power_l, int8_t target_power_r) {
  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(target_power_l, target_power_r);
  }
}

BasicDriver::BasicDriver(WheelsControl* wheels_control)
    : wheels_control_(wheels_control),
      move_type_(kInvalidMove), base_power_(0) {
}

BasicDriver::~BasicDriver() {
}

void BasicDriver::SetParam(Move move_type, int8_t base_power) {
  move_type_ = move_type;
  base_power_ = base_power;
}

void BasicDriver::Run() {
  int8_t power_l;
  int8_t power_r;

  if (move_type_ == kGoForward) {
    power_l = power_r = base_power_;
  } else if (move_type_ == kGoBackward) {
    power_l = power_r = -base_power_;
  } else if (move_type_ == kRotateLeft) {
    power_l = -base_power_;
    power_r = base_power_;
  } else if (move_type_ == kRotateRight) {
    power_l = base_power_;
    power_r = -base_power_;
  } else {
    power_l = power_r = 0;
  }

  wheels_control_->Exec(power_l, power_r);
}

void BasicDriver::Stop() {
  wheels_control_->Exec(0, 0);
}

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
  diff[curr_in] = curr_hsv;
  mv_c[curr_in] = pid_control_->CalcMv(line_trace_threshold, curr_hsv);
  curr_in +=1;
  /////////////////////////////paku//////////////////////////////
  if (move_type_ == kTraceLeftEdge) {
    mv *= -1;
  }
  get_tim(&now_time);//add
  secs[curr_in] = now_time;//add
  int8_t power_l =  static_cast<int8_t>(base_power_ + mv);//add
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);//add

  wheels_control_->Exec(power_l, power_r);
  /////////////////////////////paku//////////////////////////////

  //////////////////////////////matu/////////////////////////////
  // if (move_type_ == kTraceLeftEdge) {
  //   mv *= -1;
  // }
  // get_tim(&now_time);
  // secs[curr_in] = now_time;
  // int8_t power_l;
  // int8_t power_r;
  
  // if (move_type_ == kRightcurve){
  // power_l = static_cast<int8_t>(base_power_ + mv + 40);
  // power_r = static_cast<int8_t>(base_power_ - mv - 10 );
  // } else if(move_type_ == kLeftcurve){
  // power_l = static_cast<int8_t>(base_power_ + mv - 15 );
  // power_r = static_cast<int8_t>(base_power_ - mv + 40);
  // }else {
  // power_l = static_cast<int8_t>(base_power_ + mv);
  // power_r = static_cast<int8_t>(base_power_ - mv);
  // }
  // //  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  // //  int8_t power_r = static_cast<int8_t>(base_power_ - mv);

  //  //if (power_l == 0 && power_r == 0) {
  // //   motor_io_->StopWheels(true);
  // // } else {
  // //   motor_io_->SetWheelsPower(power_l, power_r);
  // // }
  // wheels_control_->Exec(power_l, power_r);
  //////////////////////////////matu/////////////////////////////
}

void LineTracer::Blue_Run() {
  float curr_hsv = luminous_->hsv_.v;
  mv = pid_control_->CalcMv(line_trace_blue_threshold, curr_hsv);
  mv_c[curr_in] = pid_control_->CalcMv(line_trace_threshold, curr_hsv);//add
  diff[curr_in] = curr_hsv;//add
  curr_in +=1;//add


  if (move_type_ == kTraceBlueLeftEdge) {
    mv *= -1;
  }
  get_tim(&now_time);//add
  secs[curr_in] = now_time;//add
  int8_t power_l = static_cast<int8_t>(base_power_ + mv);
  int8_t power_r = static_cast<int8_t>(base_power_ - mv);
  wheels_control_->Exec(power_l, power_r);
}

/////////////matu/////////////
void LineTracer::Vgosa() {
  get_tim(&now_time);
  // char st [256];
  // FILE* fp = fopen("Gosa1.csv", "w");
  // for (int i=0; i<curr_in; i++) {
  //   sprintf(st, "%u, %f, %f\n",secs[i], mv_c[i], diff[i]);
  //   fprintf(fp, st);
  // }
  //  fclose(fp);
  char str [256];
  char file_name[64];
  FILE* fp;

  int i = 1;
  while(true){
    snprintf(file_name,sizeof(char)*64,"ETrobo-2022-Ta/data/Gosa%i.csv",i);

    if(fp = fopen(file_name,"r")){
      fclose(fp);
    } else {
      break;
    }
    i++;
  }

  fp = fopen(file_name, "w");
  sprintf(str, "time ,mv ,diff \n");
  fprintf(fp, str);
  for (int i = 0; i < curr_in;  i++) {
    sprintf(str,"%u, %f, %f\n",secs[i], mv_c[i], diff[i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
/////////////matu/////////////

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

    default:
      break;
  }

  return end_state_;
}

DrivingManager::DrivingManager(BasicDriver* basic_driver, LineTracer* line_tracer, EndCondition* end_condition)
    : basic_driver_(basic_driver), line_tracer_(line_tracer), end_condition_(end_condition) {
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
        
    case kGoForward:
    case kGoBackward:
    case kRotateLeft:
    case kRotateRight:
      basic_driver_->Run();
      break;

    case kStopWheels:
      basic_driver_->Stop();
      break;

    default:
      break;
  }
}
