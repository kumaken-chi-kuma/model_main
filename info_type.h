#ifndef ETRC22_INFO_TYPE_H_
#define ETRC22_INFO_TYPE_H_

#include "ev3api.h"

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Hsv {
  int h;
  int s;
  int v;
};

enum Color {
  kGreen = 0,
  kBlack,
  kRed,
  kYellow,
  kBlue,
  kWhite,
  kInvalidColor,
  kColorNum
};

enum Move {
  kTraceLeftEdge = 0,
  kTraceRightEdge,
  kTraceBlueLeftEdge,
  kTraceBlueRightEdge,
  kRightcurve,
  kLeftcurve,
  kPursuitLeft,
  kPursuitRight,
  kGoForward,
  kGoBackward,
  kRotateLeft,
  kRotateRight,
  kRotateLeft_No_R,
  kRotateRight_No_L,
  kStopWheels,
  kInvalidMove,
  kRotateLeftBonus,
  kRotateRightBonus,
  kStayM
};

struct Gain {
  float kp;
  float ki;
  float kd;
};

enum End {
  kColorEnd = 0,
  kDistanceEnd,
  kThetaEnd,
  kInvalidEnd
};

struct DrivingParam {
  Move move_type;
  int8_t base_power;
  Gain gain;
  End end_type;
  Color end_color;
  float end_threshold;
  bool is_started = false; 
  bool is_finished = false;
};

#endif  // ETRC22_INFO_TYPE_H_
