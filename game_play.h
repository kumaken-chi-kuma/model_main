#ifndef ETRC22_GAME_PLAY_H_
#define ETRC22_GAME_PLAY_H_
#define FILE_NAME "test_csv.csv"

#include "info_type.h"
#include <math.h>
#include <list>

class ParamMaker {
 public:
  ParamMaker(bool R_course);
  DrivingParam MakeForward(End end_type, Color end_color, float end_threshold);
  DrivingParam MakeBackward(End end_type, Color end_color, float end_threshold);
  DrivingParam MakeRightLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown);
  DrivingParam MakeLeftLineTrace(End end_type, Color end_color, float end_threshold, bool is_carrying, bool slowdown);
  DrivingParam MakeRotateLeft(double theta, bool is_carrying, bool slowdown);
  DrivingParam MakeRotateRight(double theta, bool is_carrying, bool slowdown);
  DrivingParam MakeArmMotorMoving();
  DrivingParam MakeStopInterval();
  void JudgeTraceEdge(bool R_course);
  bool is_RightEdge_;
};

class ParamStore {
 public:
  ParamStore(Cleaning* cleaning, RouteStore* route_store, ParamMaker* param_maker);
  bool GenerateParam();
  std::list<DrivingParam> driving_params_;

 private:
  double LimitRotationAngle(double angle);
  void AddTraceParam(Robot* robot, Circle* next_circle, Direction next_direction, bool is_carrying);
  void AddPlaceParam(Robot* robot, Circle* next_circle, Direction next_direction, bool is_carrying);
  Cleaning* cleaning;
  RouteStore* route_store_;
  ParamMaker* param_maker_;
  bool is_wayback_;
  bool is_wayback_after_;
  bool is_first_RotateRight_;
  bool Rtransport;
  bool Ltransport;
};

class RouteStore {
 public:
  RouteStore(BingoArea* bingo_area, RouteSearch* route_search);
  void SaveMovingRoute(Circle* goal_circle);
  void SaveCarryRoute(Circle* goal_circle);
  std::list<char*> routes_;
  // std::list<char*> test_str;

 private:
  BingoArea* bingo_area_;
  RouteSearch* route_search_;
};

class Cleaning {
 public:
  void RouteInformation();
  Cleaning(bool kLcourse);
  ~Cleaning();
  void SolveClean();
  int route[100] = {};

  private:
  char str_tmp[1024];
};
#endif  // ETRC22_GAME_PLAY_H_
