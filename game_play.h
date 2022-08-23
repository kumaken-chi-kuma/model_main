#ifndef ETRC22_GAME_PLAY_H_
#define ETRC22_GAME_PLAY_H_
#define FILE_NAME "test_csv.csv"

#include "info_type.h"
#include <math.h>
class ParamMaker {
 public:
  ParamMaker(bool R_course);
  DrivingParam MakeForward(End end_type, Color end_color, float end_threshold);
  DrivingParam MakeBackward(End end_type, Color end_color, float end_threshold);
  DrivingParam MakeRightLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown);
  DrivingParam MakeLeftLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown);
  DrivingParam MakeRotateLeft(double theta, bool is_carrying, bool slowdown);
  DrivingParam MakeRotateRight(double theta, bool is_carrying, bool slowdown);
  DrivingParam MakeArmMotorMoving(bool is_arm_down);
  DrivingParam MakeStopInterval(int sleep_time);
  void JudgeTraceEdge(bool R_course);
  bool is_RightEdge_;
};


class Cleaning {
 public:
  void ReadDijkstraInformation();
  void Dijkstra2Moving();
  Cleaning(bool kLcourse);
  ~Cleaning();
  void StoreAllRoute();
  int route[100] = {};

  private:
  ParamMaker* param_maker;
  char str_tmp[1024];
};
#endif  // ETRC22_GAME_PLAY_H_
