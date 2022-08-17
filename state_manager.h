#ifndef ETRC22_STATE_MANAGER_H_
#define ETRC22_STATE_MANAGER_H_

#include "driving.h"

enum State {
  kTimeAttack = 0,
  kGetBonus,
  kTestRun,
  kStateNum
};

class TimeAttacker {
 public:
  TimeAttacker(DrivingManager* driving_manager, bool is_Lcourse);
  void Update();
  bool is_completed = false;

 private:
  void SetTimeAttackDriveParam(bool is_Lcourse);
  DrivingManager* driving_manager_;
};

class BonusGetter {
 public:
  BonusGetter(DrivingManager* driving_manager, bool is_Lcourse);
  void Update();

 private:
  DrivingManager* driving_manager_;
};

class StateManager {
 public:
  StateManager(TimeAttacker* time_attacker, BonusGetter* bonus_getter);
  void Update();

 private:
  void TimeAttack();
  void GetBonus();
  TimeAttacker* time_attacker_;
  BonusGetter* bonus_getter_;
  State state_;
};

#endif  // ETRC22_STATE_MANAGER_H_
