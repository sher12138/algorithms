#include "ParallelParking_in.h"

namespace neodrive {
namespace planning {

PP_ParkingIn::PP_ParkingIn() {
  Left_Right_flag = 0;
  ClearParkInit();
}

// TODO(wyc): no need
PP_ParkingIn::~PP_ParkingIn() {
  Left_Right_flag = 0;
  ClearParkInit();
}

bool PP_ParkingIn::Generate(TrajectoryPoint start_pos,
                            std::vector<TrajectoryPoint>& candidate_pos,
                            int head_tail_mode) {
  TrajectoryPoint start_pos_local(start_pos);
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  earth2vehicle(final_carport_pos_.x(), final_carport_pos_.y(),
                final_carport_pos_.theta(), start_pos.x(), start_pos.y(),
                start_pos.theta(), tmp_x, tmp_y, tmp_theta);
  start_pos_local.set_x(tmp_x);
  start_pos_local.set_y(tmp_y);
  start_pos_local.set_theta(tmp_theta);

  // TODO(wyc):
  // if (!FirstCarportCheck()) return false;
  bool bflag = false;
  bflag = FirstCarportCheck(start_pos, final_carport_pos_);
  if (bflag == false) return false;

  // check whether shift in
  vehicle_pos_.point = start_pos_local;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool shift_flag = false;
  if (Left_Right_flag == 1) {
    if (GetMaxfromCarport('y', vehicle_pos_.rect.vertex_) >=
            GetMinfromCarport('y', limit_carport_coor_) &&
        GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) <=
            GetMaxfromCarport('x', limit_carport_coor_) &&
        GetMinfromCarport('x', vehicle_pos_.rect.vertex_) >=
            GetMinfromCarport('x', limit_carport_coor_))
      shift_flag = true;
  } else {
    if (GetMinfromCarport('y', vehicle_pos_.rect.vertex_) <=
            GetMaxfromCarport('y', limit_carport_coor_) &&
        GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) <=
            GetMaxfromCarport('x', limit_carport_coor_) &&
        GetMinfromCarport('x', vehicle_pos_.rect.vertex_) >=
            GetMinfromCarport('x', limit_carport_coor_))
      shift_flag = true;
  }

  if (shift_flag == true)
    bflag = ParallelParkingShiftIn(start_pos_local, candidate_pos);
  if (bflag == false) {
    LOG_ERROR("ParallelParkingShiftIn failed");
  } else {
    bflag = ParallelParkingNormalTailIn(start_pos_local, candidate_pos);
    if (bflag == false) {
      LOG_ERROR("ParallelParkingNormalTailIn failed, try calssic tail in");
      bflag = ParallelParkingClassicTailIn(start_pos_local, candidate_pos);
      if (bflag == false) {
        LOG_ERROR("ParallelParkingClassicTailIn failed");
      }
    }
  }

  return bflag;
}

bool PP_ParkingIn::FirstCarportCheck(TrajectoryPoint& vehicle_pos,
                                     TrajectoryPoint& carspot_pos) {
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  // tranfer carport_pos to vehicle_pos
  earth2vehicle(vehicle_pos.x(), vehicle_pos.y(), vehicle_pos.theta(),
                carspot_pos.x(), carspot_pos.y(), carspot_pos.theta(), tmp_x,
                tmp_y, tmp_theta);
  // TODO(wyc):
  // Left_Right_flag = tmp_y > 0 ? 1 : 2;
  if (tmp_y > 0)
    Left_Right_flag = 1;
  else
    Left_Right_flag = 2;
  TrajectoryPoint carport_local;
  TrajectoryPoint curr_local;
  curr_local.set_x(tmp_x);
  curr_local.set_y(tmp_y);
  curr_local.set_theta(tmp_theta);
  bool bflag = false;
  bflag = pp_parking_out.Initialize(
      curr_local, carport_local, limit_carport_coor_, limit_road_oppsite_coor_,
      limit_road_rear_coor_, limit_road_front_coor_);
  if (bflag == false) {
    LOG_ERROR("pp_parking_out failed to Initialize");
  }
  return bflag;
}

/*
This function check whether vehicle is out of soft boundary of current carport;
bcllide: True-collided; False--safe
fvalue: minimum distance to the soft boundary
*/
bool PP_ParkingIn::CurrCarportBoundSafety(int forward_back_flag) {
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
      // TODO(wyc):
      // bcollide = vel_min_y <= carpot_min_y;
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
// vel_coor:current pos; tmp_radius: turn radius
// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
// target_y_val, x_range,
// TODO(wyc): reconstruct
void PP_ParkingIn::ShiftYDirection(
    TrajectoryPoint vel_coor, double left_radius, double right_radius,
    int left_right_flag, int forward_back_flag, double target_y_val,
    double x_range, std::vector<TrajectoryPoint>& in_forward_log_data,
    int& in_start_segment, int& abnomous_watcher) {
  abnomous_watcher = 0;
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward
  double tmp_radius = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int total_num = (int)(x_range / PARK_PRECISION);
  int icount = 0;
  int left_right_shift_flag = 0;  // less/larger to final pos;1:less;2:larger
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
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
  else if (flag_dir == 1)
    flag_dir = -1;
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
  origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
  old_yy_offset = vel_coor.y();
  b_carpot_flag = false;
  yy_offset = (old_yy_offset + target_y_val) * 0.5;
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
  else if (flag_dir == 1)
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
    if (left_right_shift_flag == 1 && fabs(tmp_theta) < 0.02)
      b_carpot_flag = true;
    else if (left_right_shift_flag == 2 && fabs(tmp_theta) < 0.02)
      b_carpot_flag = true;
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
  // not finished?
  if (fabs(vel_coor.y() - target_y_val) > 0.2) {
    ClassicTailInStep6(vel_coor, in_forward_log_data, in_start_segment,
                       abnomous_watcher);
  }
  return;
}

}  // namespace planning
}  // namespace neodrive
