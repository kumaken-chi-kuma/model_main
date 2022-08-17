#include "state_manager.h"

const int kLcourseParamsNum = 3;
const DrivingParam kLcourseTimeAttackParams[kLcourseParamsNum] = {
  {kTraceRightEdge, 80, {0.22, 0.013, 0.05}, kDistanceEnd, kInvalidColor, 1000, false},
  { kTraceLeftEdge, 45, { 0.22, 0, 0 }, kColorEnd, kBlue, 0},
  // { kTraceRightEdge, 50, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 1000, false},
  {kStopWheels, 0, { 0, 0, 0 }, kInvalidEnd, kInvalidColor, 0, false},
};

const int kRcourseParamsNum = 2;
const DrivingParam kRcourseTimeAttackParams[kRcourseParamsNum] = {
  { kTraceLeftEdge, 30, { 0.5, 0, 0 }, kDistanceEnd, kInvalidColor, 6400, false},
  { kGoForward, 10, { }, kDistanceEnd, kInvalidColor, 300, false},
};

TimeAttacker::TimeAttacker(DrivingManager* driving_manager, bool is_Lcourse)
    : driving_manager_(driving_manager) {
  SetTimeAttackDriveParam(is_Lcourse);
}

void TimeAttacker::SetTimeAttackDriveParam(bool is_Lcourse_) {
  if (is_Lcourse_) {
    for (int i=0; i<kLcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kLcourseTimeAttackParams[i]);
    }

  } else {
    for (int i = 0; i<kRcourseParamsNum; ++i) {
      driving_manager_->AddDrivingParam(kRcourseTimeAttackParams[i]);
    }
  }
}

void TimeAttacker::Update() {
  driving_manager_->Update();
  if (driving_manager_->DrivingParamsEmpty()) {
    is_completed = true;
  }
}

BonusGetter::BonusGetter(DrivingManager* driving_manager, bool is_Lcourse)
    : driving_manager_(driving_manager) {
}

void BonusGetter::Update() {
}

StateManager::StateManager(TimeAttacker* time_attacker, BonusGetter* bonus_getter)
    : time_attacker_(time_attacker), bonus_getter_(bonus_getter){
}

void StateManager::Update() {
  switch (state_) {
    case kTimeAttack:
      TimeAttack();
      break;

    case kGetBonus:
      GetBonus();
      break;


    default:
      break;
  }
}

void StateManager::TimeAttack() {
  time_attacker_->Update();
  if (time_attacker_->is_completed) {
    state_ = kGetBonus;
  }
}

void StateManager::GetBonus() {
  bonus_getter_->Update();
}

