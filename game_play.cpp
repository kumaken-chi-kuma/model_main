#include "game_play.h"

ParamMaker::ParamMaker(bool is_Rcourse) {
  JudgeTraceEdge(is_Rcourse);
}

void ParamMaker::JudgeTraceEdge(bool R_course) {
  if (R_course)
    is_RightEdge_ = true;
  else
    is_RightEdge_ = false;
}

DrivingParam ParamMaker::MakeForward(End end_type, Color end_color, float end_threshold) {
  DrivingParam p = {kGoForward, 40, {}, end_type, end_color, end_threshold};
  return p;
}

DrivingParam ParamMaker::MakeBackward(End end_type, Color end_color, float end_threshold) {
  DrivingParam p = { kGoBackward, 20, {}, end_type, end_color, end_threshold };
  return p;
}

DrivingParam ParamMaker::MakeRightLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown) {
  int8_t ref_power = 80;
  Gain gain = { 0.15, 0, 0.04 };
  if (slowdown) {
    ref_power = 50;
    gain = { 0.15, 0, 0.04 };
  } else if (is_carrying) {
    ref_power = 50;
    gain = { 0.18, 0, 0.04 };
  }

  DrivingParam p = { kTraceRightEdge, ref_power, gain, end_type, end_color, end_threshold };
  return p;
}

DrivingParam ParamMaker::MakeLeftLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown) {
  int8_t ref_power = 80;
  Gain gain = { 0.15, 0, 0.04 };
  if (slowdown) {
    ref_power = 50;
    gain = { 0.15, 0, 0.04 };
  } else if (is_carrying) {
    ref_power = 50;
    gain = { 0.18, 0, 0.04 };
  }
  DrivingParam p = { kTraceLeftEdge, ref_power, gain, end_type, end_color, end_threshold };
  return p;
}

DrivingParam ParamMaker::MakeRotateLeft(double theta, bool is_carrying, bool slowdown) {
  int8_t ref_power;
  Move move_type;
  End end_type;

  if (is_carrying ) {
    if (theta > M_PI/2) {
      theta *= 1.3;
      move_type = kRotateRight;
      ref_power = 50;
      end_type = kThetaEnd;
    } else {
    theta *= 0.9;
    ref_power = 40;
    move_type = kRotateLeft;
    is_RightEdge_ = true;
    end_type = kThetaEnd;
    }
  } else {
    is_RightEdge_ = false;
    if (fabs(theta) > M_PI/2) {
      theta *= 0.95;
      ref_power = 30;
    } else if (fabs(theta) > M_PI/4) {
      theta *= 0.8;
      ref_power = 50;
    } else {
      theta *= 0.75;
      ref_power = 50;
    }
    
    move_type = kRotateLeft;
    end_type = kThetaEnd;
  }

  DrivingParam p = { move_type, ref_power, { }, kThetaEnd, kInvalidColor, static_cast<float>(theta) };

  return p;
}

DrivingParam ParamMaker::MakeRotateRight(double theta, bool is_carrying, bool slowdown) {
  int8_t ref_power;
  Move move_type;
  End end_type;

  if (is_carrying) {
    if (theta < -M_PI/2) {
      theta *= 1.3;
      move_type = kRotateRight;
      ref_power = 50;
      end_type = kThetaEnd;
    } else {
    is_RightEdge_ = false;
    theta *= 0.91;
    ref_power = 40;
    move_type = kRotateRight;
    end_type = kThetaEnd;
    }
  } else {
    is_RightEdge_ = true;
    if (fabs(theta) > M_PI/2) {
      theta *= 0.90;
      ref_power = 30;
    } else if (fabs(theta) > M_PI/4) {
      theta *= 0.76;
    } else {
      theta *= 0.75;
    }
    ref_power = 50;
    move_type = kRotateRight;
    end_type = kThetaEnd;


  }

  DrivingParam p = { move_type, ref_power, { }, kThetaEnd, kInvalidColor, static_cast<float>(theta) };

  return p;
}

DrivingParam ParamMaker::MakeArmMotorMoving(bool is_arm_down){

    DrivingParam p = {};

  return p;
}

DrivingParam ParamMaker::MakeStopInterval(int sleep_time){
    
    DrivingParam p = {};

  return p;
}


Cleaning::Cleaning(bool kLcourse) {
}

Cleaning::~Cleaning() {
}

void Cleaning::StoreAllRoute() {
}

void Cleaning::ReadDijkstraInformation(){
  FILE *rp = NULL;
  rp = fopen(FILE_NAME,"r");

  while((fscanf(rp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&route[0],&route[1],&route[2],&route[3],&route[4],&route[5],&route[6],&route[7],&route[8],&route[9],&route[10],&route[11],&route[12]))!=EOF){ 
    sprintf(str_tmp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",route[0],route[1],route[2],route[3],route[4],route[5],route[6],route[7],route[8],route[9],route[10],route[11],route[12]);
    syslog(LOG_NOTICE, str_tmp);
  }
  fclose(rp);
}

void Cleaning::Dijkstra2Moving(){
    
}



