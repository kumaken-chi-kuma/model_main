#include "etrc_info.h"

Luminous::Luminous(SensorIo* sensor_io)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io) {
      SetColorReference(kInvalidColor, (Hsv){0, 0, 0});//paku
}

void Luminous::Update() {
  UpdateRgb();
  UpdateHsv();
  JudgeColor(hsv_);//paku
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb() {
  rgb_raw_t val = sensor_io_->color_rgb_raw_;
/////////////////paku//////////////////////
  int max_r = val.r - MIN_R > 0 ? val.r - MIN_R : 0;
  int max_g = val.g - MIN_G > 0 ? val.g - MIN_G : 0;
  int max_b = val.b - MIN_B > 0 ? val.b - MIN_B : 0;
  rgb_.r = (val.r < MAX_R) ? max_r * 255 / (MAX_R - MIN_R): 255;
  rgb_.g = (val.g < MAX_G) ? max_g * 255 / (MAX_G - MIN_G): 255;
  rgb_.b = (val.b < MAX_B) ? max_b * 255 / (MAX_B - MIN_B): 255;
/////////////////paku//////////////////////
}

void Luminous::UpdateHsv() {

//////////////////////////paku///////////////////////////

  int r = rgb_.r;
  int g = rgb_.g;
  int b = rgb_.b;

  max = r > g ? r : g;
  max = max > b ? max : b;
  min = r < g ? r : g;
  min < b ? min : b;

  int h;
  int s;
  int v;


  // 色相
  if(max == min) {
    h = 0;
  } else if(r == max) {
    h = 60 * (g - b) / (max - min);
  } else if(g == max) {
    h = 60 * (b - r) / (max - min) + 120;
  } else if(b == max) {
    h = 60 * (r - g) / (max - min) + 240;
  }
  // マイナスの場合の補完
  h = (h + 360) % 360;

  // 彩度
  s = (100 * (max - min)) / max;

  // 明度
  v = max;

  hsv_.h = h;
  hsv_.s = s;
  hsv_.v = v / 2;
  // maxが0の場合、黒を返す
  if(max == 0) {
    hsv_.h = 0;
    hsv_.s = 0;
    hsv_.v = 0;
  }

//////////////////////paku///////////////////////////
}


////////////////////////////////paku/////////////////////////////
Color Luminous::JudgeColor(Hsv _hsv){
  //明度が極端に低ければ、黒を返す
  if(_hsv.v < 25){
    return kBlack;
  }
  // 明度が極端に高ければ、白を返す
  if(min > 250){
    return kWhite;
  }
  // 彩度が低い場合
  if(_hsv.s < 27) {
    // 明度が低ければ、黒を返す
    if(_hsv.v < 170){
      return kBlack;
    } 
    // 明度が高ければ、白を返す
    return kWhite;
  }

  if(_hsv.h < MAX_Y_h){
    return kYellow;
  }
  if(_hsv.h < MAX_G_h){
    return kGreen;
  }
  if(_hsv.h < MAX_B_h){
    return kBlue;
  }
  if(_hsv.h < MAX_R_h){
    return kRed;
  }
  return kInvalidColor;
}

void Luminous::UpdateColor() {
  
  color_ = JudgeColor(hsv_);
    switch (color_)
  {
  case 0:
    sprintf(str_,"kGreen");
    break;
  case 1:
    sprintf(str_,"kBlack");
    break;
  case 2:
    sprintf(str_,"kRed");
    break;
  case 3:
    sprintf(str_,"kYellow");
    break;
  case 4:
    sprintf(str_,"kBlue");
    break;
  case 5:
    sprintf(str_,"kWhite");
    break;
  case 6:
    sprintf(str_,"kInvalidColor");
    break;
  case 7:
    sprintf(str_,"kColorNum");
    break;
  }
}
////////////////////////////////paku/////////////////////////////

Odometry::Odometry(MotorIo* motor_io)
  : motor_io_(motor_io) {
}

void Odometry::Update() {
  counts_r_ = motor_io_->counts_r_;
  counts_l_ = motor_io_->counts_l_;

  curr_index += 1;

  counts_rs[curr_index] = counts_r_;
  counts_ls[curr_index] = counts_l_;

  double Ll = R * (counts_ls[curr_index] - counts_ls[curr_index - 1]) * M_PI / 180;
  double Lr = R * (counts_rs[curr_index] - counts_rs[curr_index - 1]) * M_PI / 180;

  theta = (Lr - Ll) / D;
  theta_wa += theta;
  deg_theta = theta_wa * 180 / M_PI; //度に変換
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + theta / 2);
  double dy = A * sin(theta_wa + theta / 2);
  double dd = sqrt(dx * dx + dy * dy);

  before_x = x;
  before_y = y;

  x += dx;
  y += dy;

  correct_distance += A;

  difference_x = x - before_x;
  difference_y = y - before_y;
  direction = atan2(difference_y, difference_x);
}


PurePursuit::PurePursuit(MotorIo* motor_io)
  : motor_io_(motor_io), x(0), y(0), yaw(0) {
  odometry_ = new Odometry(motor_io);
  pre_point_index = INT_MAX;
}

double PurePursuit::calc_distance(double point_x, double point_y) {
  double dx = x - point_x;
  double dy = y - point_y;

  return hypot(dx, dy);
}

std::tuple<int, double> PurePursuit::search_target_index() {
  if (pre_point_index == INT_MAX) {
    std::list<int> d;

    for (int i = 0; i < kCourseParamNum; i++) {
      double dx = x - course_x[i];
      double dy = y - course_y[i];
      d.push_back(hypot(dx, dy));
    }
    std::list<int>::iterator minIt = std::min_element(d.begin(), d.end());
    ind = std::distance(d.begin(), minIt);
    pre_point_index = ind;

  } else {
    ind = pre_point_index;
    double distance = calc_distance(course_x[ind], course_y[ind]);

    while (true) {
      double next_distance = calc_distance(course_x[ind+1], course_y[ind+1]);
      if (distance < next_distance) break;
      if (ind + 1 < kCourseParamNum) {
        ind++;
      }
      distance = next_distance;
    }

    pre_point_index = ind;
  }

  while (lf > calc_distance(course_x[ind], course_y[ind])) {
    if (ind > kCourseParamNum) break;
    ind += 1;
  }

  return std::forward_as_tuple(ind, lf);
}

std::tuple<int, double> PurePursuit::pursuit_control(int pind) {
  int target_ind;
  double lf;

  std::tie(target_ind, lf) = search_target_index();

  if (pind >= target_ind) {
    target_ind = pind;
  }

  double tx,ty;
  if (target_ind < kCourseParamNum) {
      tx = course_x[target_ind];
      ty = course_y[target_ind];

  } else {
      tx = course_x[kCourseParamNum-1];
      ty = course_y[kCourseParamNum-1];
      target_ind = kCourseParamNum-1;
  }

  double alpha = atan2(ty - y, tx - x);

  return std::forward_as_tuple(target_ind, alpha);
}


void PurePursuit::Update(double odometry_x, double odometry_y) {
  odometry_->Update();
  direction_odo = odometry_->direction;
  
  if (pre_point_index == INT_MAX) {
  std::tie(target_ind, p_lf) = search_target_index();
  }

  std::tie(target_ind, delta) = pursuit_control(target_ind);

  target_distance = calc_distance(course_x[target_ind], course_y[target_ind]);
  target_direction = delta;
  difference_rad = target_direction - direction_odo;

  while (difference_rad > 4.7) //第四象限からの変換（偏角）
  {
    difference_rad = difference_rad - 6.28;
  }
  while (difference_rad < -4.7)
  {
    difference_rad = difference_rad + 6.28;
  }

  x = odometry_x;
  y = odometry_y;

}

Localize::Localize(MotorIo* motor_io) {
  odometry_ = new Odometry(motor_io);
  pure_pursuit_ = new PurePursuit(motor_io);
}

void Localize::Update() {
  odometry_->Update();
  distance_ = odometry_->correct_distance;
  theta_ = odometry_->deg_theta;
  odometry_x = odometry_->x;
  odometry_y = odometry_->y;

  pure_pursuit_->Update(odometry_x, odometry_y);
}
