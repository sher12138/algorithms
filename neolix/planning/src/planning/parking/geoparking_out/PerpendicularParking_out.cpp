#include "PerpendicularParking_out.h"

namespace neodrive {
namespace planning {
PPP_ParkingOut::PPP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}
PPP_ParkingOut::~PPP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}

bool PPP_ParkingOut::Generate(TrajectoryPoint end_pos,
                              std::vector<TrajectoryPoint>& candidate_pos,
                              int head_tail_mode) {
  // translate to carport coordinate
  TrajectoryPoint end_pos_local(end_pos);
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  earth2vehicle(final_carport_pos_.x(), final_carport_pos_.y(),
                final_carport_pos_.theta(), end_pos.x(), end_pos.y(),
                end_pos.theta(), tmp_x, tmp_y, tmp_theta);
  end_pos_local.set_x(tmp_x);
  end_pos_local.set_y(tmp_y);
  end_pos_local.set_theta(tmp_theta);

  FirstCarportCheck(end_pos, final_carport_pos_);

  double final_x = end_pos_local.x();

  double xdistoRoad =
      GetMinfromCarport('x', limit_road_oppsite_coor_) - final_x;
  double xdistoCarport =
      final_x - GetMaxfromCarport('x', limit_road_front_coor_);
  // adjust final_x
  if (fabs(xdistoRoad) + fabs(xdistoCarport) < config_limit.ppp_out_road_width)
    return false;
  if (xdistoCarport <= 3.0)
    final_x = GetMaxfromCarport('x', limit_road_front_coor_) +
              config_limit.ppp_out_min_lateral_dis_to_road;
  else if (xdistoRoad <= 3.0)
    final_x = GetMinfromCarport('x', limit_road_oppsite_coor_) -
              config_limit.ppp_out_min_lateral_dis_to_road;
  end_pos_local.set_x(final_x);

  bool bflag = false;
  if (head_tail_mode == 2)
    bflag = PerpendicularParkingTailOut(end_pos_local, candidate_pos);
  else
    bflag = PerpendicularParkingNormalOut(end_pos_local, candidate_pos);
  return bflag;
}

bool PPP_ParkingOut::FirstCarportCheck(const TrajectoryPoint& vehicle_pos,
                                       const TrajectoryPoint& carspot_pos) {
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
During turn out, this function checks whether the distance
between vehicle and caport corner is larger than 0.2m.
*/
bool PPP_ParkingOut::Satisfystep2Check() {
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
    bRoadflag = false;
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
    bRoadflag = false;
    return bRoadflag;
  } else {
    k2 = (y4 - y3) / (x4 - x3);
    b2 = (x4 * y3 - x3 * y4) / (x4 - x3);
  }
  xc = limit_carport_coor_[1].x();
  yc = limit_carport_coor_[1].y();
  if (xc <= GetMinfromCarport('x', vehicle_pos_.rect.vertex_) ||
      xc >= GetMaxfromCarport('x', vehicle_pos_.rect.vertex_))
    return false;
  if (Left_Right_flag == 1) {         // vehicle up,carport down
    yc = limit_carport_coor_[0].y();  // change yc val
    // calculate carport bound point's value
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(yc - val1, yc - val2);
    if (Portdis < 0.3) bRoadflag = true;
  } else if (Left_Right_flag == 2) {  // vehicle down,carport up
    // calculate carport bound point's value
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(val1 - yc, val2 - yc);
    if (Portdis < 0.3) bRoadflag = true;
  }
  return bRoadflag;
}
// vel_coor:current pos; tmp_radius: turn radius
// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
// target_y_val, x_range, log data
void PPP_ParkingOut::ShiftYDirection(
    TrajectoryPoint vel_coor, double left_radius, double right_radius,
    int left_right_flag, int forward_back_flag, double target_y_val,
    double x_range, std::vector<TrajectoryPoint>& in_forward_log_data,
    int& in_start_segment) {
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward
  double tmp_radius = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int total_num = (int)(x_range / PARK_PRECISION);
  int icount = 0;
  int left_right_shift_flag = 0;  // less/larger to final pos;1:less;2:larger
  if (vel_coor.y() < target_y_val)
    left_right_shift_flag = 1;
  else
    left_right_shift_flag = 2;
  // 1. right-forward
  if (left_right_flag == 1) {  // left->right
    flag = -1;
    tmp_radius = left_radius;
  } else if (left_right_flag == 2) {  // right->left
    flag = 1;
    tmp_radius = right_radius;
  }
  if (forward_back_flag == 0)
    flag_dir = 1;
  else if (forward_back_flag == 1)
    flag_dir = -1;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
  origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
  old_yy_offset = vel_coor.y();
  b_carpot_flag = false;
  // save data define
  vel_coor.set_kappa(-flag / tmp_radius);           // curvature
  vel_coor.set_direction((flag_dir == 1 ? 0 : 1));  // forward
  vel_coor.set_segment_index(in_start_segment);     // segment index
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    icount++;
    tmp_x += (flag_dir * PARK_PRECISION);
    tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
    tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    RegulateYaw(tmp_theta);
    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    yy_offset = (old_yy_offset + target_y_val) * 0.5;
    if (left_right_shift_flag == 1 && tmp_y >= yy_offset)
      b_carpot_flag = true;
    else if (left_right_shift_flag == 2 && tmp_y <= yy_offset)
      b_carpot_flag = true;
    bboardlimit = CarportCurrBoundSafety();
    if (bboardlimit == true) {
      in_forward_log_data.pop_back();  // give up the previous point
      break;
    }
    if (icount >= total_num * 0.5) b_carpot_flag = true;
    in_forward_log_data.push_back(vel_coor);
  }
  in_start_segment++;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];

  // 2. left-forward
  if (left_right_flag == 1) {  // left->right
    flag = 1;
    tmp_radius = right_radius;
  } else if (left_right_flag == 2) {  // right->left
    flag = -1;
    tmp_radius = left_radius;
  }
  if (forward_back_flag == 0)
    flag_dir = 1;
  else if (forward_back_flag == 1)
    flag_dir = -1;
  origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
  origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
  b_carpot_flag = false;
  // save data define
  vel_coor.set_kappa(-flag / tmp_radius);           // curvature
  vel_coor.set_direction((flag_dir == 1 ? 0 : 1));  // forward
  vel_coor.set_segment_index(in_start_segment);     // segment index
  tmp_x = vel_coor.x();
  tmp_y = vel_coor.y();
  tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    icount++;
    tmp_x += (flag_dir * PARK_PRECISION);
    tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
    tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    RegulateYaw(tmp_theta);
    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    if (fabs(tmp_theta) < 0.02) b_carpot_flag = true;
    bboardlimit = CarportCurrBoundSafety();
    if (bboardlimit == true) {
      in_forward_log_data.pop_back();  // give up the previous point
      break;
    }
    if (icount >= total_num) b_carpot_flag = true;
    // save data
    in_forward_log_data.push_back(vel_coor);
  }
  in_start_segment++;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  return;
}
// vel_coor:current pos; tmp_radius: turn radius
// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
// target_y_val, x_range, log data
void PPP_ParkingOut::ShiftYDirectionTailOut(
    TrajectoryPoint vel_coor, double left_radius, double right_radius,
    int left_right_flag, int forward_back_flag, double target_y_val,
    double x_range, std::vector<TrajectoryPoint>& in_forward_log_data,
    int& in_start_segment) {
  int flag = 0;
  double tmp_radius = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int total_num = (int)(x_range / PARK_PRECISION);
  int icount = 0;
  int left_right_shift_flag = 0;  // less/larger to final pos;1:less;2:larger
  if (vel_coor.y() < target_y_val)
    left_right_shift_flag = 1;
  else
    left_right_shift_flag = 2;
  // 1. right-forward
  if (left_right_flag == 1) {  // left->right
    flag = -1;
    tmp_radius = left_radius;
  } else if (left_right_flag == 2) {  // right->left
    flag = 1;
    tmp_radius = right_radius;
  }
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
  origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();

  old_yy_offset = vel_coor.y();
  b_carpot_flag = false;
  // save data define
  vel_coor.set_kappa(-flag / tmp_radius);        // curvature
  vel_coor.set_direction(forward_back_flag);     // forward
  vel_coor.set_segment_index(in_start_segment);  // segment index
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    icount++;
    tmp_x += PARK_PRECISION;
    tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
    tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    RegulateYawHpiHpi(tmp_theta);
    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    yy_offset = (old_yy_offset + target_y_val) * 0.5;
    if (left_right_shift_flag == 1 && tmp_y >= yy_offset)
      b_carpot_flag = true;
    else if (left_right_shift_flag == 2 && tmp_y <= yy_offset)
      b_carpot_flag = true;
    bboardlimit = CarportCurrBoundSafety();
    if (bboardlimit == true) {
      in_forward_log_data.pop_back();  // give up the previous point
      break;
    }
    if (icount >= total_num * 0.5) b_carpot_flag = true;
    in_forward_log_data.push_back(vel_coor);
  }
  in_start_segment++;
  if (in_forward_log_data.size() < 1) return;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];

  // 2. left-forward
  if (left_right_flag == 1) {  // left->right
    flag = 1;
    tmp_radius = right_radius;
  } else if (left_right_flag == 2) {  // right->left
    flag = -1;
    tmp_radius = left_radius;
  }
  origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
  origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
  b_carpot_flag = false;
  // save data define
  vel_coor.set_kappa(-flag / tmp_radius);        // curvature
  vel_coor.set_direction(forward_back_flag);     // forward
  vel_coor.set_segment_index(in_start_segment);  // segment index
  tmp_x = vel_coor.x();
  tmp_y = vel_coor.y();
  tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    icount++;
    tmp_x += PARK_PRECISION;
    tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
    tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    RegulateYawHpiHpi(tmp_theta);
    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    if (fabs(fabs(tmp_theta) - M_PI) < 0.02) b_carpot_flag = true;
    bboardlimit = CarportCurrBoundSafety();
    if (bboardlimit == true) {
      in_forward_log_data.pop_back();  // give up the previous point
      break;
    }
    if (icount >= total_num) b_carpot_flag = true;
    // save data
    in_forward_log_data.push_back(vel_coor);
  }
  in_start_segment++;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  return;
}
/*
During turn out, this function checks whether the distance
between vehicle and caport corner is larger than 0.2m.
*/
bool PPP_ParkingOut::SatisfyTailOutStep2Check() {
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
    bRoadflag = false;
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
    bRoadflag = false;
    return bRoadflag;
  } else {
    k2 = (y4 - y3) / (x4 - x3);
    b2 = (x4 * y3 - x3 * y4) / (x4 - x3);
  }
  if (Left_Right_flag == 1) {  // vehicle up,carport down
                               // calculate carport bound point's value
    xc = limit_carport_coor_[0].x();
    yc = limit_carport_coor_[0].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(val1 - yc, val2 - yc);
    if (Portdis < 0.3) bRoadflag = true;
  } else if (Left_Right_flag == 2) {  // vehicle down,carport up
                                      // calculate carport bound point's value
    xc = limit_carport_coor_[0].x();
    yc = limit_carport_coor_[0].y();
    val1 = k1 * xc + b1;
    val2 = k2 * xc + b2;
    // check carport
    Portdis = fmin(yc - val1, yc - val2);
    if (Portdis < 0.3) bRoadflag = true;
  }
  return bRoadflag;
}

}  // namespace planning
}  // namespace neodrive
