#include "etrc_info.h"

#include "app.h"
#include "device_io.h"
#include "driving.h"
#include "game_play.h"
#include "state_manager.h"
#include "ev3api.h"


bool kLcourse = true;

MotorIo* motor_io;
SensorIo* sensor_io;
Luminous* luminous;
Localize* localize;
WheelsControl* wheels_control;
BasicDriver* basic_driver;
LineTracer* line_tracer;
EndCondition* end_condition;
DrivingManager* driving_manager;
TimeAttacker* time_attacker;
BonusGetter* bonus_getter;
Cleaning* cleaning;
StateManager* state_manager;

static void initialize() {
  motor_io = new MotorIo();
  sensor_io = new SensorIo();
  luminous = new Luminous(sensor_io);
  localize = new Localize(motor_io);
  wheels_control = new WheelsControl(motor_io);
  basic_driver = new BasicDriver(wheels_control);
  line_tracer = new LineTracer(wheels_control, luminous);
  end_condition = new EndCondition(luminous, localize);
  driving_manager = new DrivingManager(basic_driver, line_tracer, end_condition);
  time_attacker = new TimeAttacker(driving_manager, kLcourse);
  bonus_getter = new BonusGetter(driving_manager, kLcourse);
  cleaning = new Cleaning(kLcourse);
  state_manager = new StateManager(time_attacker, bonus_getter);
}

static void finalize() {
  delete state_manager;
  delete cleaning;
  delete bonus_getter;
  delete time_attacker;
  delete driving_manager;
  delete end_condition;
  delete line_tracer;
  delete basic_driver;
  delete wheels_control;
  delete localize;
  delete luminous;
  delete sensor_io;
  delete motor_io;
}

void main_task(intptr_t unused) {
  initialize();
  sta_cyc(UPDATE_INFO_CYC);

/*
 while (true) {
   if(ev3_button_is_pressed(button_t LEFT_BUTTON)){
    break;
    }
    else if (ev3_button_is_pressed(button_t RIGHT_BUTTON)){
      kLcourse = false;
    }
  }
*/
  while (true) {
    if (sensor_io->left_button_pressed_) break;
     else if (sensor_io->right_button_pressed_) {
       kLcourse = false;
       break;
     }
    tslp_tsk(TASK_INTERVAL_DT_MS*1000U);
  }

  // char str[264];
  // sprintf(str, "%s", kLcourse);
  // syslog(LOG_NOTICE, str);

  tslp_tsk(TASK_INTERVAL_DT_MS*500000U);

  while (true) {
    if (sensor_io->ultrasonic_sensor_distance_ > START_URLTRASONIC_DIST_THRESHOLD) break;
    tslp_tsk(TASK_INTERVAL_DT_MS*1000U);
  }


  tslp_tsk(START_INTERVAL_DT_MS*1000U);

  sta_cyc(EXEC_ACTION_CYC);

  tslp_tsk(TASK_INTERVAL_DT_MS*1000U);

  while (true) {
    if (sensor_io->enter_button_pressed_) break;
    tslp_tsk(100*1000U);
  }

  stp_cyc(EXEC_ACTION_CYC);
  stp_cyc(UPDATE_INFO_CYC);
  finalize();
  ext_tsk();
}

void exec_action_task(intptr_t unused) {
  localize->Update();
  state_manager->Update();
  ext_tsk();
}

void update_info_task(intptr_t unused) {
  motor_io->Update();
  sensor_io->Update();
  luminous->Update();
  // localize->Update();
  ext_tsk();
}

void solve_bingo_task(intptr_t unused) {
  cleaning->SolveClean();
  ext_tsk();
}
