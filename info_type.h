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

// enum BlockId {
//     kK1 = 0,
//     kR1,
//     kR2,
//     kY1,
//     kY2,
//     kB1,
//     kB2,
//     kG1,
//     kG2,
//     kInvalidBlockId,
//     kBlockIdNum
// };

// enum Direction {
//     kEast = 0,
//     kNorthEast,
//     kNorth,
//     kNorthWest,
//     kWest,
//     kSouthWest,
//     kSouth,
//     kSouthEast,
//     kInvalidDirection,
//     kDirectionNum,
// };

// struct Circle {
//   char id;
//   int x;
//   int y;
//   char color;
//   Circle* next[kNextToMax];
//   Block* block;
//   int cost;
//   Circle* prev;
//   bool queue_added;
//   int next_num;
//   Block* initial_block;
// };

// struct Block {
//   BlockId id;
//   char color;
//   Circle* circle;
//   Circle* target;
//   bool carrying_completed;
// };

// struct Robot {
//   Circle* circle;
//   Direction direction;
// };

#endif  // ETRC22_INFO_TYPE_H_
