#include "PerpendicularParking_in.h"

namespace neodrive {
namespace planning {
PPP_ParkingIn::PPP_ParkingIn() {
  Left_Right_flag = 0;
  ClearParkInit();
}
PPP_ParkingIn::~PPP_ParkingIn() {
  Left_Right_flag = 0;
  ClearParkInit();
}

bool PPP_ParkingIn::Generate(TrajectoryPoint start_pos,
                             std::vector<TrajectoryPoint>& candidate_pos,
                             int head_tail_mode) {
  // translate to carport coordinate
  TrajectoryPoint start_pos_local(start_pos);
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  earth2vehicle(final_carport_pos_.x(), final_carport_pos_.y(),
                final_carport_pos_.theta(), start_pos.x(), start_pos.y(),
                start_pos.theta(), tmp_x, tmp_y, tmp_theta);
  RegulateYaw(tmp_theta);
  start_pos_local.set_x(tmp_x);
  start_pos_local.set_y(tmp_y);
  start_pos_local.set_theta(tmp_theta);

  bool bflag = false;

  bflag = FirstCarportCheck(start_pos, final_carport_pos_);
  if (bflag == false) return false;

  double final_x = start_pos_local.x();
  double xdistoRoad =
      GetMinfromCarport('x', limit_road_oppsite_coor_) - final_x;
  double xdistoCarport =
      final_x - GetMaxfromCarport('x', limit_road_front_coor_);
  // maybe we should adapte this val to config
  if (fabs(xdistoRoad) + fabs(xdistoCarport) < 4.8) return false;
  //  if (fabs(limit_carport_coor_[0].y()) + fabs(limit_carport_coor_[1].y())
  //  < 3.8)
  //    return false;

  bool shift_flag = false;
  // check whether shift in
  vehicle_pos_.point = start_pos_local;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  if (fabs(start_pos_local.theta()) < 0.3 ||
      fabs(fabs(start_pos_local.theta()) - M_PI) < 0.3) {
    if (GetMaxfromCarport('y', limit_carport_coor_) >=
            GetMaxfromCarport('y', vehicle_pos_.rect.vertex_) &&
        GetMinfromCarport('y', limit_carport_coor_) <=
            GetMinfromCarport('y', vehicle_pos_.rect.vertex_))
      shift_flag = true;
  }

  if (shift_flag == true) {
    if (head_tail_mode == 2)  // head in
      ;
    else  // tail in
      bflag = PerpendicularParkingShiftTailIn(start_pos_local, candidate_pos);
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingShiftTailIn failed");
    }
    return bflag;
  }
  if (head_tail_mode == 2) {  // head in
    bflag = PerpendicularParkingNormalHeadIn(start_pos_local, candidate_pos);
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingNormalHeadIn failed, try classic head in");
      bflag = PerpendicularParkingClassicHeadIn(start_pos_local, candidate_pos);
      if (bflag == false) {
        LOG_ERROR("PerpendicularParkingClassicHeadIn failed");
      }
    }
  } else if (head_tail_mode == 1) {
    bflag = PerpendicularParkingNormalTailIn(start_pos_local, candidate_pos);
    if (bflag == false)
      LOG_ERROR("PerpendicularParkingNormalTailIn failed, try classic tail in");
    bflag = PerpendicularParkingClassicTailIn(start_pos_local, candidate_pos);
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingClassicTailIn failed");
    }
  } else {
    bflag = PerpendicularParkingNormalTailIn(start_pos_local, candidate_pos);
    head_tail_mode = 1;
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingNormalTailIn failed, try classic tail in");
      bflag = PerpendicularParkingClassicTailIn(start_pos_local, candidate_pos);
      if (bflag == false) {
        LOG_ERROR("PerpendicularParkingClassicTailIn failed");
      }
    }
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingClassicTailIn failed, try normal head in");
      bflag = PerpendicularParkingNormalHeadIn(start_pos_local, candidate_pos);
      head_tail_mode = 2;
    }
    if (bflag == false) {
      LOG_ERROR("PerpendicularParkingNormalHeadIn failed, try calssic head in");
      bflag = PerpendicularParkingClassicHeadIn(start_pos_local, candidate_pos);
      if (bflag == false) {
        LOG_ERROR("PerpendicularParkingClassicHeadIn failed");
      }
    }
  }

  return bflag;
}

bool PPP_ParkingIn::FirstCarportCheck(TrajectoryPoint& vehicle_pos,
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
  TrajectoryPoint carport_local;
  TrajectoryPoint curr_local;
  curr_local.set_x(tmp_x);
  curr_local.set_y(tmp_y);
  curr_local.set_theta(tmp_theta);
  bool bflag = false;
  bflag = ppp_parking_out.Initialize(
      curr_local, carport_local, limit_carport_coor_, limit_road_oppsite_coor_,
      limit_road_rear_coor_, limit_road_front_coor_);
  if (bflag == false) {
    LOG_ERROR("ppp_parking_out failed to Initialize");
  }
  return bflag;
}

// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
// target_y_val, x_range
void PPP_ParkingIn::ShiftYDirection(
    TrajectoryPoint vel_coor, double left_radius, double right_radius,
    int left_right_flag, int forward_back_flag, double target_y_val,
    double x_range, std::vector<TrajectoryPoint>& in_forward_log_data,
    int& in_start_segment, int& abnomous_watcher) {
  abnomous_watcher = 0;
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward
  double tmp_radius = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int i_abnormal_old_watcher = 0, i_abnormal_watcher = 0;
  int total_num = (int)(x_range / PARK_PRECISION);
  int icount = 0;
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
    if (left_right_flag == 1 && tmp_y >= yy_offset)
      b_carpot_flag = true;
    else if (left_right_flag == 2 && tmp_y <= yy_offset)
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
  i_abnormal_old_watcher = in_forward_log_data.size();

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
  // not finished?
  if (fabs(vel_coor.y() - target_y_val) >= 0.2) {
    ClassicTailInStep4(vel_coor, in_forward_log_data, in_start_segment,
                       abnomous_watcher);
  }
  return;
}

void PPP_ParkingIn::ShiftYDirectionHeadIn(
    TrajectoryPoint vel_coor, double left_radius,
    double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
    int turn_leri_flag, int left_right_vehicle,
    int forward_back_flag,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
    double target_y_val, double x_range,
    std::vector<TrajectoryPoint>&
        in_forward_log_data,  // target_y_val, x_range, log data
    int& in_start_segment, int& abnomous_watcher) {  // log abnomous_watcher
  abnomous_watcher = 0;
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward
  double tmp_radius = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int i_abnormal_old_watcher = 0, i_abnormal_watcher = 0;
  int total_num = (int)(x_range / PARK_PRECISION);
  int icount = 0;
  // 1. right-forward
  if (turn_leri_flag == 1) {  // left->right
    flag = -1;
    tmp_radius = left_radius;
  } else if (turn_leri_flag == 2) {  // right->left
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
  vel_coor.set_kappa(-flag / tmp_radius);        // curvature
  vel_coor.set_direction(forward_back_flag);     // forward
  vel_coor.set_segment_index(in_start_segment);  // segment index
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();

  while (!b_carpot_flag) {
    icount++;
    tmp_x += (flag_dir * PARK_PRECISION);
    if (left_right_vehicle == 2) {
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    } else {
      tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    }
    RegulateYawHpiHpi(tmp_theta);

    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    yy_offset = (old_yy_offset + target_y_val) * 0.5;
    if (turn_leri_flag == 1 && tmp_y >= yy_offset)
      b_carpot_flag = true;
    else if (turn_leri_flag == 2 && tmp_y <= yy_offset)
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
  i_abnormal_old_watcher = in_forward_log_data.size();

  // 2. left-forward
  if (turn_leri_flag == 1) {  // left->right
    flag = 1;
    tmp_radius = right_radius;
  } else if (turn_leri_flag == 2) {  // right->left
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
  vel_coor.set_kappa(-flag / tmp_radius);        // curvature
  vel_coor.set_direction(forward_back_flag);     // forward
  vel_coor.set_segment_index(in_start_segment);  // segment index
  tmp_x = vel_coor.x();
  tmp_y = vel_coor.y();
  tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    icount++;
    tmp_x += (flag_dir * PARK_PRECISION);
    if (left_right_vehicle == 2) {
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    } else {
      tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    }
    RegulateYawHpiHpi(tmp_theta);
    vel_coor.set_x(tmp_x);
    vel_coor.set_y(tmp_y);
    vel_coor.set_theta(tmp_theta);
    vehicle_pos_.point = vel_coor;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    if (fabs(tmp_theta) >= M_PI - 0.05) b_carpot_flag = true;
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
  if (fabs(vel_coor.y() - target_y_val) >= 0.2) {
    ClassicHeadInStep4(vel_coor, in_forward_log_data, in_start_segment,
                       abnomous_watcher);
  }
  return;
}
/*
During turn out, this function checks whether the distance
between vehicle and caport corner is larger than 0.2m.
*/
bool PPP_ParkingIn::SatisfyHeadinStep3Check() {
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

void PPP_ParkingIn::MoveXDirectionHeadIn(
    TrajectoryPoint start_vel_pos, TrajectoryPoint end_vel_pos,
    std::vector<TrajectoryPoint>& forward_log_data, int& start_segment,
    int head_direction) {
  if (fabs(start_vel_pos.x() - end_vel_pos.x()) < PARK_PRECISION) return;
  TrajectoryPoint curr_vel_pos(start_vel_pos);
  double x_start = curr_vel_pos.x(), xx_start = x_start;
  double y_start = curr_vel_pos.y(), theta_start = curr_vel_pos.theta();
  double x_end = end_vel_pos.x();
  double y_end = end_vel_pos.y(), theta_end = end_vel_pos.theta();
  double delta_theta = 0.0, temp_b = 0.0, delta_y = 0.0;
  int inum = (int)floor(fabs(x_end - x_start) / PARK_PRECISION);
  if (inum > 1) {
    delta_theta = (theta_end - theta_start) / inum;
    delta_y = (y_end - y_start) / inum;
  }
  double tmp_x = curr_vel_pos.x(), tmp_y = curr_vel_pos.y(),
         tmp_theta = curr_vel_pos.theta();
  tmp_theta = atan2(y_end - y_start, x_end - x_start);
  RegulateYawHpiHpi(tmp_theta);
  temp_b = y_start - tan(tmp_theta) * x_start;
  // saved data
  curr_vel_pos.set_theta(tmp_theta);
  curr_vel_pos.set_kappa(0);  // curvature
  if (head_direction == 2)
    curr_vel_pos.set_direction(1);  // backward
  else
    curr_vel_pos.set_direction(0);                // forward
  curr_vel_pos.set_segment_index(start_segment);  // segment index

  if (x_start < x_end - PARK_PRECISION) {  // go forward
    while (xx_start < x_end) {
      tmp_x += PARK_PRECISION;
      tmp_y = delta_y + forward_log_data[forward_log_data.size() - 1].y();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYawHpiHpi(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      xx_start = tmp_x;
    }
  } else if (x_start > x_end + PARK_PRECISION) {  // go backward
    while (xx_start > x_end) {
      tmp_x -= PARK_PRECISION;
      tmp_y = delta_y + forward_log_data[forward_log_data.size() - 1].y();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYawHpiHpi(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      xx_start = tmp_x;
    }
  }
  start_segment++;
  return;
}

}  // namespace planning
}  // namespace neodrive
