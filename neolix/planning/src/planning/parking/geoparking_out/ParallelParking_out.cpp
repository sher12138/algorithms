#include "ParallelParking_out.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

PP_ParkingOut::PP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}
PP_ParkingOut::~PP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}

/*main function for parallel parking out.
1. first check the space. if dis to carport and road less than 0.6, do not
support. (This means the road is less than 3.5m, not enough space to turn out.)
(**future work can adjust space with carport space**)
2. call ParallelParkingNormalOut to generate park-out trajectory.
*/
bool PP_ParkingOut::Generate(TrajectoryPoint start_pos,
                             std::vector<TrajectoryPoint>& candidate_pos,
                             int head_tail_mode) {
  // translate to carport coordinate
  TrajectoryPoint start_pos_local(start_pos);
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  earth2vehicle(final_carport_pos_.x(), final_carport_pos_.y(),
                final_carport_pos_.theta(), start_pos.x(), start_pos.y(),
                start_pos.theta(), tmp_x, tmp_y, tmp_theta);
  start_pos_local.set_x(tmp_x);
  start_pos_local.set_y(tmp_y);
  start_pos_local.set_theta(tmp_theta);

  FirstCarportCheck(start_pos, final_carport_pos_);

  double final_y = start_pos_local.y();

  if (Left_Right_flag == 2) {
    double ydistoRoad =
        GetMinfromCarport('y', limit_road_oppsite_coor_) - final_y;
    double ydistoCarport =
        final_y - GetMaxfromCarport('y', limit_road_front_coor_);
    if (fabs(ydistoRoad) < protected_half_width &&
        fabs(ydistoCarport) < protected_half_width) {
      LOG_ERROR("given pos to road or carport is too close");
      return false;
    }

    if (ydistoRoad < protected_half_width)
      final_y = GetMinfromCarport('y', limit_road_oppsite_coor_) -
                protected_half_width;
    else if (ydistoCarport < protected_half_width)
      final_y =
          GetMaxfromCarport('y', limit_road_front_coor_) + protected_half_width;
  } else if (Left_Right_flag == 1) {
    double ydistoRoad =
        final_y - GetMaxfromCarport('y', limit_road_oppsite_coor_);
    double ydistoCarport =
        GetMinfromCarport('y', limit_road_front_coor_) - final_y;
    if (fabs(ydistoRoad) < protected_half_width &&
        fabs(ydistoCarport) < protected_half_width) {
      LOG_ERROR("given pos to road or carport is too close");
      return false;
    }
    if (ydistoRoad < protected_half_width)
      final_y = GetMaxfromCarport('y', limit_road_oppsite_coor_) +
                protected_half_width;
    else if (ydistoCarport < protected_half_width)
      final_y =
          GetMinfromCarport('y', limit_road_front_coor_) - protected_half_width;
  }
  start_pos_local.set_y(final_y);
  bool bflag = false;
  bflag = ParallelParkingNormalOut(start_pos_local, candidate_pos);
  return bflag;
}
/*
check whether turn left out or turn right out.
Left_Right_flag=2:turn left out;
Left_Right_flag=1:turn right out;
*/
bool PP_ParkingOut::FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                                      TrajectoryPoint& carspot_pos) {
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  // tranfer carport_pos to vehicle_pos
  earth2vehicle(vehicle_pos.x(), vehicle_pos.y(), vehicle_pos.theta(),
                carspot_pos.x(), carspot_pos.y(), carspot_pos.theta(), tmp_x,
                tmp_y, tmp_theta);
  if (tmp_y > 0)
    Left_Right_flag = 1;
  else
    Left_Right_flag = 2;
  return true;
}

/*
This function check whether current vehicle pos have enough space to turn out.
*/
bool PP_ParkingOut::Satisfystep2Check() {
  bool bflag = false;
  double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
  double y1 = 0.0, y2 = 0.0, y3 = 0.0, y4 = 0.0;
  double k1 = 0.0, b1 = 0.0, k2 = 0.0, b2 = 0.0;
  double xc = 0.0, yc = 0.0, val1 = 0.0, val2 = 0.0;
  double gateval = 0.0;
  // line 1
  x1 = vehicle_pos_.rect.vertex_[2].x();
  y1 = vehicle_pos_.rect.vertex_[2].y();
  x2 = vehicle_pos_.rect.vertex_[1].x();
  y2 = vehicle_pos_.rect.vertex_[1].y();
  if (fabs(x2 - x1) < 0.1) {
    bflag = true;
    return bflag;
  } else {
    k1 = (y2 - y1) / (x2 - x1);
    b1 = (x2 * y1 - x1 * y2) / (x2 - x1);
  }
  // line 2
  x3 = vehicle_pos_.rect.vertex_[0].x();
  y3 = vehicle_pos_.rect.vertex_[0].y();
  x4 = vehicle_pos_.rect.vertex_[3].x();
  y4 = vehicle_pos_.rect.vertex_[3].y();
  if (fabs(x3 - x4) < 0.1) {
    bflag = true;
    return bflag;
  } else {
    k2 = (y4 - y3) / (x4 - x3);
    b2 = (x4 * y3 - x3 * y4) / (x4 - x3);
  }

  double flex_length = 1.5 * VehicleParam::Instance()->length();
  if (Left_Right_flag == 2) {  // vehicle up,carport down
                               // calculate carport bound point's value
    xc = limit_carport_coor_[1].x();
    yc = limit_carport_coor_[1].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check
    gateval = 0.5;
    // more than 45 degree or getval
    if (k1 > tan(45 * M_PI / 180) && k2 > tan(45 * M_PI / 180) &&
        (val1 - yc > gateval) && (val2 - yc > gateval)) {
      bflag = true;
      return bflag;
    }
    gateval = 1.0;
    double carport_length = 0.0;
    carport_length =
        fmin(limit_carport_coor_[1].x(), limit_carport_coor_[2].x()) -
        fmax(limit_carport_coor_[0].x(), limit_carport_coor_[3].x());
    if (carport_length >= flex_length)
      gateval = gateval + (carport_length - flex_length) * 0.2;
    else
      gateval = gateval + (flex_length - carport_length) * 0.2;
    if ((val1 - yc > gateval) && (val2 - yc > gateval))
      bflag = true;
    else
      bflag = false;
  } else if (Left_Right_flag == 1) {  // vehicle down,carport up
                                      // calculate carport bound point's value
    xc = limit_carport_coor_[1].x();
    yc = limit_carport_coor_[1].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check
    gateval = 0.5;
    // more than 45 degree or getval
    if (k1 < tan(-45 * M_PI / 180) && k2 < tan(-45 * M_PI / 180) &&
        (yc - val1 > gateval) && (yc - val2 > gateval)) {
      bflag = true;
      return bflag;
    }
    gateval = 1.0;
    double carport_length = 0.0;
    carport_length =
        fmin(limit_carport_coor_[1].x(), limit_carport_coor_[2].x()) -
        fmax(limit_carport_coor_[0].x(), limit_carport_coor_[3].x());
    if (carport_length >= flex_length)
      gateval = gateval + (carport_length - flex_length) * 0.2;
    else
      gateval = gateval + (flex_length - carport_length) * 0.2;
    if ((yc - val1 > gateval) && (yc - val2 > gateval))
      bflag = true;
    else
      bflag = false;
  }
  return bflag;
}
/*
During turn out, this function checks whether the distance
between vehicle and caport corner is larger than 0.2m.
*/
bool PP_ParkingOut::Satisfystep4Check() {
  double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
  double y1 = 0.0, y2 = 0.0, y3 = 0.0, y4 = 0.0;
  double k1 = 0.0, b1 = 0.0, k2 = 0.0, b2 = 0.0;
  double xc = 0.0, yc = 0.0, val1 = 0.0, val2 = 0.0;

  bool bRoadflag = false;
  double Portdis = 1000;

  // line 1
  x1 = vehicle_pos_.rect.vertex_[2].x();
  y1 = vehicle_pos_.rect.vertex_[2].y();
  x2 = vehicle_pos_.rect.vertex_[1].x();
  y2 = vehicle_pos_.rect.vertex_[1].y();
  if (fabs(x2 - x1) < 0.1) {
    bRoadflag = true;
    return bRoadflag;
  } else {
    k1 = (y2 - y1) / (x2 - x1);
    b1 = (x2 * y1 - x1 * y2) / (x2 - x1);
  }
  // line 2
  x3 = vehicle_pos_.rect.vertex_[0].x();
  y3 = vehicle_pos_.rect.vertex_[0].y();
  x4 = vehicle_pos_.rect.vertex_[3].x();
  y4 = vehicle_pos_.rect.vertex_[3].y();
  if (fabs(x3 - x4) < 0.1) {
    bRoadflag = true;
    return bRoadflag;
  } else {
    k2 = (y4 - y3) / (x4 - x3);
    b2 = (x4 * y3 - x3 * y4) / (x4 - x3);
  }
  if (Left_Right_flag == 2) {  // vehicle up,carport down
                               // calculate carport bound point's value
    xc = limit_carport_coor_[1].x();
    yc = limit_carport_coor_[1].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(val1 - yc, val2 - yc);
    if (Portdis < 0.3) bRoadflag = true;
  } else if (Left_Right_flag == 1) {  // vehicle down,carport up
                                      // calculate carport bound point's value
    xc = limit_carport_coor_[1].x();
    yc = limit_carport_coor_[1].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(yc - val1, yc - val2);
    if (Portdis < 0.3) bRoadflag = true;
  }
  return bRoadflag;
}
/*
This function check whether vehicle is out of soft boundary of current carport;
bcllide: True-collided; False--safe
fvalue: minimum distance to the soft boundary
*/
bool PP_ParkingOut::CurrCarportBoundSafety(int forward_back_flag) {
  bool bcollide = false;
  double fvalue = -100.0;
  if (limit_carport_coor_.size() < 4) return false;
  double carpot_min_x =
      fmax(limit_carport_coor_[0].x(), limit_carport_coor_[3].x());
  double carpot_max_x =
      fmin(limit_carport_coor_[1].x(), limit_carport_coor_[2].x());
  double carpot_min_y = GetMinfromCarport('y', limit_carport_coor_);
  double carpot_max_y = GetMaxfromCarport('y', limit_carport_coor_);

  double vel_min_y = GetMinfromCarport('y', vehicle_pos_.rect.vertex_);
  double vel_max_y = GetMaxfromCarport('y', vehicle_pos_.rect.vertex_);
  double vel_min_x = GetMinfromCarport('x', vehicle_pos_.rect.vertex_);
  double vel_max_x = GetMaxfromCarport('x', vehicle_pos_.rect.vertex_);

  if (Left_Right_flag == 2) {      // vehicle up, carport down
    if (forward_back_flag == 0) {  // forward
      if (vel_min_y <= carpot_min_y)
        bcollide = true;
      else if (vel_max_x >= carpot_max_x)
        bcollide = true;
    } else if (forward_back_flag == 1) {  // backward
      if (vel_min_y <= carpot_min_y)
        bcollide = true;
      else if (vel_min_x <= carpot_min_x)
        bcollide = true;
    }
    fvalue = fabs(vel_min_y - carpot_min_y);
    fvalue = fmin(fvalue, fabs(vel_min_x - carpot_min_x));
    fvalue = fmin(fvalue, fabs(carpot_max_x - vel_max_x));
  } else if (Left_Right_flag == 1) {  // vehicle down, carport up
    if (forward_back_flag == 0) {     // forward
      if (vel_max_y >= carpot_max_y)
        bcollide = true;
      else if (vel_max_x >= carpot_max_x)
        bcollide = true;
    } else if (forward_back_flag == 1) {  // backward
      if (vel_max_y >= carpot_max_y)
        bcollide = true;
      else if (vel_min_x <= carpot_min_x)
        bcollide = true;
    }
    fvalue = fabs(vel_max_y - carpot_max_y);
    fvalue = fmin(fvalue, fabs(vel_min_x - carpot_min_x));
    fvalue = fmin(fvalue, fabs(carpot_max_x - vel_max_x));
  }
  return bcollide;
}

}  // namespace planning
}  // namespace neodrive
