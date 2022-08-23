#include "game_play.h"

ParamMaker::ParamMaker(bool is_Rcourse) {
  JudgeTraceEdge(R_course);
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
      move_type = kRotateBlockRight;
      ref_power = 50;
      end_type = kThetaEndB;
    } else {
    theta *= 0.9;
    ref_power = 40;
    move_type = kRotateBlockLeft;
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
      move_type = kRotateBlockRight;
      ref_power = 50;
      end_type = kThetaEndB;
    } else {
    is_RightEdge_ = false;
    theta *= 0.91;
    ref_power = 40;
    move_type = kRotateBlockRight;
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

DrivingParam ParamMaker::MakeArmMotorMoving(){

}

DrivingParam ParamMaker::MakeStopInterval(){}

double ParamStore::LimitRotationAngle(double angle) {
  double ret_angle = angle;
  if (fabs(angle) > M_PI) {
    if (angle >= 0)
      ret_angle =  angle - 2*M_PI;
    else
      ret_angle =  angle + 2*M_PI;
  }
  return ret_angle;
}

void ParamStore::AddTraceParam(Robot* robot, Circle* next_circle, Direction next_direction, bool is_carrying) {
  // bool slowdown = (next_circle->block != NULL) ? true : false;
  bool slowdown = false;
  // ブロック運搬直後の動き
  if (is_wayback_after_) {
    is_wayback_after_ = false;
    // 回転が必要なとき
    if (robot->direction != next_direction) {
      double from = static_cast<int>(robot->direction) * M_PI / 4;
      double to = static_cast<int>(next_direction) * M_PI / 4;
      double dtheta = LimitRotationAngle(to - from);
      // 左回転のとき
      if (dtheta > 0) {

        if (dtheta > M_PI/2){
          driving_params_.push_back(param_maker_->MakeRotateLeft(dtheta, is_carrying, slowdown));
        } else {
          driving_params_.push_back(param_maker_->MakeRotateLeft(dtheta, is_carrying, slowdown));
        }

        bool slowdown = (next_circle->block != NULL) ? true : false;
        next_circle->block = NULL;
        driving_params_.push_back(param_maker_->MakeForwardOutCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
        switch (next_circle->color) {
          case 'R':
            driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
            break;
          case 'G':
            driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
            break;
          case 'B':
            driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
            break;
          case 'Y':
            driving_params_.push_back(param_maker_->MakeLeftLineTrace(kDistanceEnd, kInvalidColor, -50, is_carrying, slowdown));
            driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
            break;

          default:
            break;
        }
      // 右回転のとき
      } else { 
        if (dtheta < -M_PI/2) {
          driving_params_.push_back(param_maker_->MakeRotateRight(dtheta, is_carrying, slowdown));
        } else {
          driving_params_.push_back(param_maker_->MakeRotateRight(dtheta, is_carrying, slowdown));
        }
        bool slowdown = (next_circle->block != NULL) ? true : false;
        next_circle->block = NULL;
        driving_params_.push_back(param_maker_->MakeForwardOutCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
        switch (next_circle->color) {
          case 'R':
            driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
            break;
          case 'G':
            driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
            break;
          case 'B':
            driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
            break;
          case 'Y':
            driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
            break;

          default:
            break;

        }
      }
    }
  // 交点サークル間移動
  } else {
    // 回転が必要なとき
    if (robot->direction != next_direction) {
      double from = static_cast<int>(robot->direction) * M_PI / 4;
      double to = static_cast<int>(next_direction) * M_PI / 4;
      double dtheta = LimitRotationAngle(to - from);
      // 左回転のとき
      if (is_first_RotateRight_) {
        bool slowdown = (next_circle->block != NULL) ? true : false;
        next_circle->block = NULL;
        driving_params_.push_back(param_maker_->MakeForward2Center(kDistanceEndB, kInvalidColor, is_carrying, slowdown));
        //最初の右回転
        driving_params_.push_back(param_maker_->MakeRotateRight(dtheta, is_carrying, slowdown));
        is_first_RotateRight_ = false;
        driving_params_.push_back(param_maker_->MakeForwardOutCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
        driving_params_.push_back(param_maker_->MakeFirstRightLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
      } else {
        if (dtheta > 0) {
          driving_params_.push_back(param_maker_->MakeForward2Center(kDistanceEndB, kInvalidColor, is_carrying, slowdown));
          driving_params_.push_back(param_maker_->MakeRotateLeft(dtheta, is_carrying, slowdown));
          bool slowdown = (next_circle->block != NULL) ? true : false;
          next_circle->block = NULL;
          driving_params_.push_back(param_maker_->MakeForwardOutCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
          if (param_maker_->is_RightEdge_) {
              switch (next_circle->color) {
                case 'R':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                  break;
                case 'G':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                  break;
                case 'B':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                  break;
                case 'Y':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                  break;

                default:
                  break;
              }
            } else {
              switch (next_circle->color) {
                case 'R':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                  break;
                case 'G':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                  break;
                case 'B':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                  break;
                case 'Y':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                  break;

                default:
                  break;
              }
            }
          // switch (next_circle->color) {
          //   case 'R':
          //     driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
          //     break;
          //   case 'G':
          //     driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
          //     break;
          //   case 'B':
          //     driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
          //     break;
          //   case 'Y':
          //     driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
          //     break;

          //   default:
          //     break;
          // }
      // 右回転のとき
        } else {
            //２回目以降の右回転
            driving_params_.push_back(param_maker_->MakeForward2Center(kDistanceEndB, kInvalidColor, is_carrying, slowdown));
            driving_params_.push_back(param_maker_->MakeRotateRight(dtheta, is_carrying, slowdown));
            driving_params_.push_back(param_maker_->MakeForwardOutCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
            bool slowdown = (next_circle->block != NULL) ? true : false;
            next_circle->block = NULL;
            if (param_maker_->is_RightEdge_) {
              switch (next_circle->color) {
                case 'R':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                  break;
                case 'G':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                  break;
                case 'B':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                  break;
                case 'Y':
                  driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                  break;

                default:
                  break;
              }
            } else {
              switch (next_circle->color) {
                case 'R':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                  break;
                case 'G':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                  break;
                case 'B':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                  break;
                case 'Y':
                  driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                  break;

                default:
                  break;
              }
            }
          }
        }     
    } else {
      //交点サークル上を通過するための直進
        driving_params_.push_back(param_maker_->MakeForwardPassCircle(kDistanceEnd, kInvalidColor, is_carrying, slowdown));
          if (param_maker_->is_RightEdge_) {
            bool slowdown = (next_circle->block != NULL) ? true : false;
            next_circle->block = NULL;
            switch (next_circle->color) {
              case 'R':
                driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                break;
              case 'G':
                driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                break;
              case 'B':
                driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                break;
              case 'Y':
                driving_params_.push_back(param_maker_->MakeRightLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                break;

              default:
                break;
            }
          } else {
            bool slowdown = (next_circle->block != NULL) ? true : false;
            next_circle->block = NULL;
            switch (next_circle->color) {
              case 'R':
                driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kRed, 0, is_carrying, slowdown));
                break;
              case 'G':
                driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kGreen, 0, is_carrying, slowdown));
                break;
              case 'B':
                driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kBlue, 0, is_carrying, slowdown));
                break;
              case 'Y':
                driving_params_.push_back(param_maker_->MakeLeftLineTrace(kColorEnd, kYellow, 0, is_carrying, slowdown));
                break;

              default:
                break;
            }
          }
    }
  }

  robot->direction = next_direction;
  robot->circle = next_circle;
}

void ParamStore::AddPlaceParam(Robot* robot, Circle* next_circle, Direction next_direction, bool is_carrying) {
  // bool slowdown = (next_circle->block != NULL) ? true : false;
  bool slowdown = true;
  if (is_wayback_) {
    //設置後後退
    if (Rtransport) {
      driving_params_.push_back(param_maker_->MakeRotateBlockBRight());
      // driving_params_.push_back(param_maker_->MakeForward(kDistanceEndB, kInvalidColor, 50));
      Rtransport = false;
    } else if(Ltransport){
      driving_params_.push_back(param_maker_->MakeRotateBlockBLeft());
      // driving_params_.push_back(param_maker_->MakeForward(kDistanceEndB, kInvalidColor, 50));
      Rtransport = false;
    } else {
      switch (next_circle->color) {
        case 'R':
          driving_params_.push_back(param_maker_->MakeBackward(kDistanceEnd, kInvalidColor, -50));
          driving_params_.push_back(param_maker_->MakeBackward(kColorEndB, kRed, 0));
          break;
        case 'G':
          driving_params_.push_back(param_maker_->MakeBackward(kColorEndB, kGreen, 0));
          break;
        case 'B':
          driving_params_.push_back(param_maker_->MakeBackward(kColorEndB, kBlue, 0));
          break;
        case 'Y':
          driving_params_.push_back(param_maker_->MakeBackward(kColorEndB, kYellow, 0));
          break;
        
        default:
          break;
        
      }
    }
    robot->circle = next_circle;
    is_wayback_ = false;
    is_wayback_after_ = true;
  } else {
    //設置動作
    driving_params_.push_back(param_maker_->MakePlaceForward2Center());

    if (robot->direction != next_direction) {
      double from = static_cast<int>(robot->direction) * M_PI / 4;
      double to = static_cast<int>(next_direction) * M_PI / 4;
      double dtheta = LimitRotationAngle(to - from);
      if (dtheta< M_PI/2 && dtheta > 0) {
        driving_params_.push_back(param_maker_->MakeRotateLeft(dtheta));

        driving_params_.push_back(param_maker_->MakeForward(kDistanceEndB, kInvalidColor, 57));
        switch (next_circle->color) {
          case 'R':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kRed, 0));
            break;
          case 'G':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kGreen, 0));
            break;
          case 'B':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kBlue, 0));
            break;
          case 'Y':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kYellow, 0));
            
            break;
          default:
            break;
        }
        robot->direction = next_direction;
      } else if (dtheta > M_PI/2){
        driving_params_.push_back(param_maker_->MakeRotateLeft(dtheta, is_carrying, slowdown));
        Ltransport = true;
      } else if (dtheta < 0 && dtheta > -M_PI/2){
        driving_params_.push_back(param_maker_->MakeRotateRight(dtheta));

        driving_params_.push_back(param_maker_->MakeForward(kDistanceEndB, kInvalidColor, 57));
        switch (next_circle->color) {
          case 'R':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kRed, 0));
            break;
          case 'G':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kGreen, 0));
            break;
          case 'B':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kBlue, 0));
            break;
          case 'Y':
            driving_params_.push_back(param_maker_->MakeForward(kColorEndB, kYellow, 0));
            
            break;
          default:
            break;
        }
        robot->direction = next_direction;
      } else {
        driving_params_.push_back(param_maker_->MakeRotateRight(dtheta, is_carrying, slowdown));
        Rtransport = true;
      }
    }
    robot->circle = next_circle;
    is_wayback_ = true;
  }
}

RouteStore::RouteStore(BingoArea* bingo_area, RouteSearch* route_search)
    : bingo_area_(bingo_area), route_search_(route_search) {
}

void RouteStore::SaveMovingRoute(Circle* goal_circle) {
  Circle* curr_circle = bingo_area_->robot_.circle;
  Circle* back_circle = NULL;
  // char str[32] = {};
  //test_str.push_back(&goal_circle->id);

  char str[kRouteCharNum] = {};
  for (int i = 0; i < kRouteCharNum - 2; ++i) {
    str[i] = curr_circle->id;
    back_circle = curr_circle;
    curr_circle = curr_circle->prev;
    if (curr_circle->id == goal_circle->id) {
      str[i + 1] = curr_circle->id;
      str[i + 2] = '\0';
      break;
    }
  }
  char* route = new char(strlen(str) + 1);
  strcpy(route, str);
  routes_.push_back(route);
  syslog(LOG_NOTICE, route);

  if (goal_circle->id == '0')
    route_search_->reverse_circle_ = NULL;
  else
    route_search_->reverse_circle_ = back_circle;
}

void RouteStore::SaveCarryRoute(Circle* goal_circle) {
  Circle* curr_circle = bingo_area_->robot_.circle;
  Circle* back_circle = NULL;

  char str[kRouteCharNum] = {};
  for (int i = 0; i < kRouteCharNum - 3; ++i) {
    str[i] = curr_circle->id;
    back_circle = curr_circle;
    curr_circle = curr_circle->prev;
    if (curr_circle->id == goal_circle->id) {
      str[i + 1] = curr_circle->id;
      str[i + 2] = back_circle->id;
      str[i + 3] = '\0';
      break;
    }
  }
  char* route = new char(strlen(str) + 1);
  strcpy(route, str);
  routes_.push_back(route);
  syslog(LOG_NOTICE, route);


}

Cleaning::Cleaning(bool kLcourse) {
}

Cleaning::~Cleaning() {
}

void Cleaning::SolveClean() {
}

void Cleaning::RouteInformation(){
  FILE *rp = NULL;
  rp = fopen(FILE_NAME,"r");

  while((fscanf(rp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&route[0],&route[1],&route[2],&route[3],&route[4],&route[5],&route[6],&route[7],&route[8],&route[9],&route[10],&route[11],&route[12]))!=EOF){ 
    sprintf(str_tmp,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",route[0],route[1],route[2],route[3],route[4],route[5],route[6],route[7],route[8],route[9],route[10],route[11],route[12]);
    syslog(LOG_NOTICE, str_tmp);
  }
  fclose(rp);
}



