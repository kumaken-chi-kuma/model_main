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
  // rgb_.r = val.r;
  // rgb_.g = val.g;
  // rgb_.b = val.b;
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
  hsv_.v = v;
  // maxが0の場合、黒を返す
  if(max == 0) {
    hsv_.h = 0;
    hsv_.s = 0;
    hsv_.v = 0;
  }

  // char str[264];//add
  // sprintf(str, "  \n", r, g, b);//add
  // syslog(LOG_NOTICE, str);//add
//////////////////////paku///////////////////////////


  ////////////////////matu///////////////////////////
  // float r = static_cast<float>(rgb_.r);
  // float g = static_cast<float>(rgb_.g);
  // float b = static_cast<float>(rgb_.b);

  // float max = r > g ? r : g;
  // max = max > b ? max : b;
  // float min = r < g ? r : g;
  // min = min < b ? min : b;
  // float c = max - min;

  // float h;
  // if (c == 0) {
  //   h = -1;
  // } else if (max == r) {
  //   h = fmodf(((g - b) / c), 6);
  // } else if (max == g) {
  //   h = ((b - r) / c) + 2;
  // } else if (max == b) {
  //   h = ((r - g) / c) + 4;
  // } else {
  //   h = -1;
  // }

  // if (h != -1) {
  //   h = 60 * h;
  // }

  // float s;
  // if (max == 0) {
  //   s = 0;
  // } else {
  //   s = c / max;
  // }

  // float v = max;

  // hsv_.h = h;
  // hsv_.s = s * 100;
  // hsv_.v = v/2;//change
  // // char str[264];//add
  // sprintf(str, " h: %f s: %f v: %f \n",hsv_.h, hsv_.s, hsv_.v);//add
  // syslog(LOG_NOTICE, str);//add
  ////////////////////matu///////////////////////////
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
  if(_hsv.s < 20) {
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
  char str_[1024];
  
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
  // char str[256];
  // sprintf(str, " h: %d s: %d v: %d %s",hsv_.h, hsv_.s, hsv_.v, str_);
  // syslog(LOG_NOTICE, str);
}
////////////////////////////////paku/////////////////////////////

// void Luminous::UpdateColor() {
// }

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
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + theta / 2);
  double dy = A * sin(theta_wa + theta / 2);
  double dd = sqrt(dx * dx + dy * dy);

  before_x = x;
  before_y = y;

  x += dx;
  y += dy;
  distance += dd;

  difference_x = x - before_x;
  difference_y = y - before_y;
  direction = atan2(difference_y, difference_x);

  char a[264];
  sprintf(a, "distance: %f\n", distance);
  syslog(LOG_NOTICE, a);
}

// CubicSpline::CubicSpline() {
//   setCourseParam();
// }
/*
void CubicSpline::setCourseParam() {
  int data = kCourseParamNum - 1;

  for (int i = 0; i <= data; i++) {
    a_.push_back(y[i]);
  }

  for (int i = 0; i < data; i++) {
    if (i == 0) {
      c_.push_back(0.0);
    } else if (i == data) {
      c_.push_back(0.0);
    } else {
      c_.push_back(3.0 * (a_[i-1] - 2.0 * a_[i] + a_[i+1]));
    }
  }

  for (int i = 0; i < data; i++) {
    if (i == 0) {
      w_.push_back(0.0);
    } else {
      double tmp = 4.0 - w_[i-1];
      c_[i] = (c_[i] - c_[i-1]) / tmp;
      w_.push_back(1.0 / tmp);
    }
  }

  for (int i = (data-1); i > 0; i--){
      c_[i] = c_[i] - c_[i+1] * w_[i];
  }

  for (int i = 0; i <= data; i++) {
    if (i == data) {
      d_.push_back(0.0);
      b_.push_back(0.0);

    } else {
      d_.push_back((c_[i+1] - c_[i]) / 3.0);
      b_.push_back(a_[i+1] - a_[i] - c_[i] - d_[i]);
    }
  }
}

double CubicSpline::CalcEndpoint(const std::list<double> y){
    int dt = y.size();

    double dy = b_[j] + (c_[j] + d_[j] * dt) * dt;
    return dy * dy;

    return 0;
}

double CubicSpline::Calc(double t) {
  int j = int(floor(t));
  if (j < 0) {
    j = 0;
  } else if(j >= a_.size()) {
    j = a_.size() - 1;
  }

  double dt = t - j;
  double result = a_[j] + (b_[j] + (c_[j] + d_[j] * dt) * dt) * dt;
  accl = 2 * c_[j] + 6 * d_[j] * dt;

  return result;
}
*/

PurePursuit::PurePursuit()
  : x(0), y(0), yaw(0) {
  // cubic_spline_ = new CubicSpline();
  readTargetCourseCoordinate();
  pre_point_index = INT_MAX;
}

void PurePursuit::readTargetCourseCoordinate() {
  // for (int i=0; i<size; i++) {
  //   course_x[i] = ;
  //   course_y[i] = ;
  // }
}

double PurePursuit::calc_distance(double point_x, double point_y) {
  double dx = x - point_x;
  double dy = y - point_y;

  return hypot(dx, dy);
}


std::tuple<int, double> PurePursuit::search_target_index() {
  int ind;
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
    double distance = calc_distance(course_x[ind],course_y[ind]);

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
  double lf;
  int target_ind;
  x = odometry_x;
  y = odometry_y;

  std::tie(target_ind, lf) = search_target_index();

  double delta;
  std::tie(target_ind, delta) = pursuit_control(target_ind);
}

Localize::Localize(MotorIo* motor_io) {
  odometry_ = new Odometry(motor_io);
  pure_pursuit_ = new PurePursuit();
}

void Localize::Update() {
  odometry_->Update();
  distance_ = odometry_->distance;
  theta_ = odometry_->theta;
  odometry_x = odometry_->x;
  odometry_y = odometry_->y;

  //pure_pursuit_->Update(odometry_x, odometry_y);
}
