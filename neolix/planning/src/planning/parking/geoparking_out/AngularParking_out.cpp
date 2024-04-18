#include "AngularParking_out.h"
namespace neodrive {
namespace planning {

AP_ParkingOut::AP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}
AP_ParkingOut::~AP_ParkingOut() {
  Left_Right_flag = 0;
  ClearParkInit();
}

/*main function for parallel parking out.
1. first check the space. if dis to carport and road less than 0.6, do not
support. (This means the road is less than 3.5m, not enough space to turn out.)
(**future work can adjust space with carport space**)
2. call Parallel_Parking_Normal_Out to generate park-out trajectory.
*/
bool AP_ParkingOut::Generate(TrajectoryPoint end_pos,
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

  bool bflag = false;
  bflag = AngularParkingNormalHeadOut(end_pos_local, candidate_pos);
  /*if (head_tail_mode == 2)
          bflag = AngularParkingNormalTailOut(end_pos_local, candidate_pos);
  else
          bflag = AngularParkingNormalHeadOut(end_pos_local,
  candidate_pos);*/
  return bflag;
}
/*
check whether turn left out or turn right out.
Left_right_flag=1:turn left out;
Left_right_flag=2:turn right out;
*/
bool AP_ParkingOut::FirstCarportCheck(const TrajectoryPoint& vehicle_pos,
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

void AP_ParkingOut::ShiftYDirection(
    TrajectoryPoint vel_coor, double left_radius,
    double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
    int left_right_flag,
    int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
    double target_y_val, double x_range,
    std::vector<TrajectoryPoint>&
        in_forward_log_data,  // target_y_val, x_range, log data
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
void AP_ParkingOut::ShiftYDirectionTailOut(
    TrajectoryPoint vel_coor, double left_radius,
    double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
    int left_right_flag,
    int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
    double target_y_val, double x_range,
    std::vector<TrajectoryPoint>&
        in_forward_log_data,  // target_y_val, x_range, log data
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

}  // namespace planning
}  // namespace neodrive
