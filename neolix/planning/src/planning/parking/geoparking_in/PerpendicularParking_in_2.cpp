#include "PerpendicularParking_in.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

bool PPP_ParkingIn::PerpendicularParkingClassicTailIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: perpendicular to the carport*/
  ClassicTailInStep1(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step2: adjust vehicle to constant position*/
  // xdirection [1.0~1.5m] to the forward road
  // ydirection [-0.2~0.2m] to the forward carport
  ClassicTailInStep2(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  ///*step3:turn-back to a certain angle*/
  ClassicTailInStep3(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step4:Shift to x-axis*/
  ClassicTailInStep4(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step5:go back to carport(0,0,0)*/
  ClassicTailInStep5(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  // get point
  size_t forward_log_data_size = forward_log_data.size();
  for (size_t i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}

// perpendicular to the carport
void PPP_ParkingIn::ClassicTailInStep1(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*step1: adjust vehicle parallel to y-axis*/
  double tmp_theta = vel_pos.theta();
  if (Left_Right_flag == 1)
    RegulateYaw0Pi(tmp_theta);
  else if (Left_Right_flag == 2)
    RegulateYawPi0(tmp_theta);
  if (fabs(fabs(tmp_theta) - M_PI_2) <= 0.05) {  // already parallel
    return;
  }
  vel_pos.set_theta(tmp_theta);
  // need to be parallel
  double ver_dis_to_carport = 0.0;
  double ver_dis_to_roadLimit = 0.0;
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);

  ver_dis_to_carport = fabs(GetMinfromCarport('x', vehicle_pos_.rect.vertex_) -
                            GetMaxfromCarport('x', limit_road_front_coor_));
  ver_dis_to_roadLimit =
      fabs(GetMinfromCarport('x', limit_road_oppsite_coor_) -
           GetMaxfromCarport('x', vehicle_pos_.rect.vertex_));

  double temp_radius = 0.0;
  int imode = 0;
  if (ver_dis_to_carport >
      ver_dis_to_roadLimit) {  // close to road, need to be away from road
    temp_radius = left_radius;
    if (Left_Right_flag == 1) {
      if (vel_pos.theta() > M_PI_2)  // right-forward
        imode = 4;
      else  // right-backward
        imode = 3;
    } else if (Left_Right_flag == 2) {
      if (vel_pos.theta() < -M_PI_2)  // left-forward
        imode = 1;
      else  // left-backward
        imode = 2;
    }
  } else {  // close to carport, need to be away from carport
    temp_radius = right_radius;
    if (Left_Right_flag == 1) {
      if (vel_pos.theta() > M_PI_2)  // left-backward
        imode = 2;
      else  // left-forward
        imode = 1;
    } else if (Left_Right_flag == 2) {
      if (vel_pos.theta() < -M_PI_2)  // right-backward
        imode = 3;
      else  // right-forward
        imode = 4;
    }
  }
  switch (imode) {
    case 1:  // left-forward
      Step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 1, 0,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 2:  // left-backward
      Step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 1, 1,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 3:  // right-backward
      Step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 2, 1,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 4:  // right-forward
      Step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 2, 0,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    default:
      break;
  }
  return;
}

/*
This function make the vehicle parallel to the carport.
*/
// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
void PPP_ParkingIn::Step1ParalleltoYAxis(
    TrajectoryPoint vel_coor, double left_radius, double right_radius,
    int left_right_flag, int forward_back_flag,
    std::vector<TrajectoryPoint>& in_forward_log_data, int& in_start_segment,
    int& abnomous_watcher) {
  bool bboardlimit = false;
  abnomous_watcher = 0;
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  // begin to turn parallel
  int b_carpot_flag = 0;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  int flag = 0;
  int dirction_flag = 1;
  if (Left_Right_flag == 2) dirction_flag = -1;

  double tmp_radius = 0.0, origin_x = 0.0, origin_y = 0.0;
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    //(1)turn
    while (1) {
      if (left_right_flag == 2) {
        flag = -1;
        tmp_radius = right_radius;
      } else {
        flag = 1;
        tmp_radius = left_radius;
      }

      origin_x = -flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      if (forward_back_flag == 1)
        tmp_y -= dirction_flag * PARK_PRECISION;
      else
        tmp_y += dirction_flag * PARK_PRECISION;
      if (Left_Right_flag == 1) {
        tmp_x =
            flag * sqrt(tmp_radius * tmp_radius - pow((tmp_y - origin_y), 2)) +
            origin_x;
        tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
        RegulateYaw0Pi(tmp_theta);
      } else if (Left_Right_flag == 2) {
        tmp_x =
            -flag * sqrt(tmp_radius * tmp_radius - pow((tmp_y - origin_y), 2)) +
            origin_x;
        tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
        RegulateYawPi0(tmp_theta);
      }
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      vel_coor.set_kappa(flag / tmp_radius);
      if (forward_back_flag == 0)
        vel_coor.set_direction(0);
      else
        vel_coor.set_direction(1);
      vel_coor.set_segment_index(in_start_segment);
      in_forward_log_data.push_back(vel_coor);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        // in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // check whether continue
      if (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.05) {
        b_carpot_flag = 1;
        break;
      }
    }
    if (b_carpot_flag == 1) break;
    in_start_segment++;
    i_abnormal_old_watcher = in_forward_log_data.size();
    //(2)change manuaver,continue
    while (1) {
      if (left_right_flag == 2) {
        flag = 1;
        tmp_radius = left_radius;
      } else {
        flag = -1;
        tmp_radius = right_radius;
      }
      origin_x = -flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      if (forward_back_flag == 1)
        tmp_y += dirction_flag * PARK_PRECISION;
      else
        tmp_y -= dirction_flag * PARK_PRECISION;
      if (Left_Right_flag == 1) {
        tmp_x =
            flag * sqrt(tmp_radius * tmp_radius - pow((tmp_y - origin_y), 2)) +
            origin_x;
        tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
        RegulateYaw0Pi(tmp_theta);
      } else if (Left_Right_flag == 2) {
        tmp_x =
            -flag * sqrt(tmp_radius * tmp_radius - pow((tmp_y - origin_y), 2)) +
            origin_x;
        tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
        RegulateYawPi0(tmp_theta);
      }
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      vel_coor.set_kappa(flag / tmp_radius);
      if (forward_back_flag == 0)
        vel_coor.set_direction(1);
      else
        vel_coor.set_direction(0);
      vel_coor.set_segment_index(in_start_segment);
      in_forward_log_data.push_back(vel_coor);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        // in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // check whether continue
      if (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.05) {
        b_carpot_flag = 1;
        break;
      }
    }
    if (b_carpot_flag == 1) break;
    in_start_segment++;
    i_abnormal_watcher = in_forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  in_start_segment++;
  return;
}

void PPP_ParkingIn::ClassicTailInStep2(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  TrajectoryPoint curr_vel_pos(forward_log_data[forward_log_data.size() - 1]);
  double final_x_val = curr_vel_pos.x();
  double new_final_x_val = 0.0;
  /*step2: adjust vehicle to constant position*/
  // xdirection [1.2~1.6m] to the forward road left
  // ydirection [-0.2~0.2m] to the forward road
  double xdistoforwardroad =
      final_x_val - GetMaxfromCarport('x', limit_road_front_coor_);
  xdistoforwardroad -= 1.2;
  if ((xdistoforwardroad <= 1.6) && (xdistoforwardroad >= 1.2))  // no change
    new_final_x_val = final_x_val;
  else
    new_final_x_val =
        GetMaxfromCarport('x', limit_road_front_coor_) + 1.2 + 1.6;
  int head_dir = 1;
  if (Left_Right_flag == 2) head_dir = 2;
  // check y shift or not?
  if (final_x_val > new_final_x_val + 0.2) {  // shift left
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
  } else if (final_x_val < new_final_x_val - 0.2) {  // shift right
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return;

  // check y shift or not?
  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = curr_vel_pos.x();
  tmp_y = left_radius - 0.3;
  tmp_theta = M_PI_2;
  if (Left_Right_flag == 2) {
    tmp_y = -right_radius + 0.3;
    tmp_theta = -M_PI_2;
  }
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);

  head_dir = 1;
  if (Left_Right_flag == 1 && curr_vel_pos.y() < end_vel_pos.y())
    head_dir = 1;
  else if (Left_Right_flag == 1 && curr_vel_pos.y() > end_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && curr_vel_pos.y() < end_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && curr_vel_pos.y() > end_vel_pos.y())
    head_dir = 1;
  MoveYDirection(curr_vel_pos, end_vel_pos, forward_log_data, start_segment,
                 head_dir);
  return;
}

void PPP_ParkingIn::ClassicTailInStep3(
    TrajectoryPoint& vel_coor, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  bool bboardlimit = false;
  abnomous_watcher = 0;
  vel_coor = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward;
  double tmp_radius = 0.0;
  bool step_3_flag = true;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;

  while (step_3_flag) {
    // 1. right-forward
    if (Left_Right_flag == 1) {  // left-back
      flag = -1;
      tmp_radius = fmax(left_radius, fabs(vel_coor.y()));
    } else if (Left_Right_flag == 2) {  // right-back
      flag = 1;
      tmp_radius = fmax(right_radius, fabs(vel_coor.y()));
      flag_dir = -1;
    }
    origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
    origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
    // save data defiine
    vel_coor.set_kappa(-flag / tmp_radius);     // curvature
    vel_coor.set_direction(1);                  // backward
    vel_coor.set_segment_index(start_segment);  // segment index
    b_carpot_flag = false;
    double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
           tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {
      // first half of segment, move y
      if (fabs(tmp_y) > tmp_radius * 0.5) {
        if (Left_Right_flag == 1) {
          tmp_y -= (flag_dir * PARK_PRECISION);
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        } else if (Left_Right_flag == 2) {
          tmp_y -= (flag_dir * PARK_PRECISION);
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        }
      } else {  // second half of segmemt,move x
        tmp_x -= PARK_PRECISION;
        tmp_x = fmax(0, tmp_x);
        tmp_y =
            flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw(tmp_theta);
      }

      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        forward_log_data.pop_back();  // give up the previous point
        break;
      }
      if (fabs(vel_coor.theta()) <= 0.05) {
        b_carpot_flag = true;
        step_3_flag = false;
      }
      forward_log_data.push_back(vel_coor);
    }
    start_segment++;
    vel_coor = forward_log_data[forward_log_data.size() - 1];
    if (step_3_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    // not finished?
    // 2. go backward
    double temp_k = tan(vel_coor.theta());
    double temp_b = vel_coor.y() - temp_k * vel_coor.x();
    vel_coor.set_kappa(0);                      // curvature
    vel_coor.set_direction(1);                  // backward
    vel_coor.set_segment_index(start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    b_carpot_flag = false;
    double delta_x = 0.0;
    while (!b_carpot_flag) {
      // insert straight line
      delta_x += PARK_PRECISION;
      tmp_x -= PARK_PRECISION;
      tmp_y = temp_k * tmp_x + temp_b;

      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag == true) break;
      // save data
      forward_log_data.push_back(vel_coor);
      if (delta_x >= 0.5) b_carpot_flag = true;
    }
    start_segment++;
    vel_coor = forward_log_data[forward_log_data.size() - 1];
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 1) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}
// turn&shift to x-axis
void PPP_ParkingIn::ClassicTailInStep4(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool bboardlimit = false;
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  if (fabs(vel_pos.y()) <= PARK_PRECISION) return;
  // need to shift y
  //(2):go forward to a certain position, 0.4m to forward limit
  double shift_y = vel_pos.y();
  double distofront = 0.0;
  distofront = GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) - vel_pos.x();
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = GetMinfromCarport('x', limit_road_oppsite_coor_) -
          2 * PARK_PRECISION - distofront;
  tmp_y = vel_pos.y();
  tmp_theta = 0.0;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int head_dir = 1;
  if (vel_pos.x() < end_vel_pos.x())
    head_dir = 1;
  else
    head_dir = 2;
  MoveXDirection(vel_pos, end_vel_pos, forward_log_data, start_segment,
                 head_dir);
  //(3)backward, turn
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  double x_range = 0.0;
  x_range = GetMinfromCarport('x', limit_road_oppsite_coor_) -
            GetMinfromCarport('x', limit_carport_coor_);
  x_range -= (GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) -
              GetMinfromCarport('x', vehicle_pos_.rect.vertex_));
  if (x_range <= 1.0) {
    abnomous_watcher = 1;
    return;
  }
  x_range -= 0.8;
  if (shift_y > 0) {  // shift down
    ShiftYDirection(vel_pos, left_radius, right_radius, 2, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  } else {  // shift up
    ShiftYDirection(vel_pos, left_radius, right_radius, 1, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return;
  return;
}

// straight backward
void PPP_ParkingIn::ClassicTailInStep5(
    TrajectoryPoint& vel_coor, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  vel_coor = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool bboardlimit = false;
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();

  tmp_x = 0.0;
  tmp_y = vel_coor.y();
  tmp_theta = 0.0;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int head_dir = 1;
  if (vel_coor.x() < end_vel_pos.x())
    head_dir = 1;
  else
    head_dir = 2;
  MoveXDirection(vel_coor, end_vel_pos, forward_log_data, start_segment,
                 head_dir);
  return;
}
bool PPP_ParkingIn::PerpendicularParkingNormalTailIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: perpendicular to the carport*/
  ClassicTailInStep1(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /**********************************************
  Generate from carport origin
  ***********************************************/
  std::vector<TrajectoryPoint> backward_log_data;
  TrajectoryPoint back_vel_pos;
  // clear the init para
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  bool bflag = false;
  bflag = ppp_parking_out.Generate(back_vel_pos, backward_log_data, 1);
  if (bflag == false) return false;
  /*step5: calculate x-direction move or not*/
  // check whether x-direction OK?
  double new_final_x = backward_log_data[backward_log_data.size() - 1].x();
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];

  int head_dir = 1;
  if (Left_Right_flag == 2) head_dir = 2;
  // check x shift or not?
  if (back_vel_pos.x() > new_final_x + 0.2) {  // shift left
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
  } else if (back_vel_pos.x() < new_final_x - 0.2) {  // shift right
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return false;
  /*step 6: check y-direction move or not*/
  // check y shift or not?
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  back_vel_pos = backward_log_data[backward_log_data.size() - 1];
  head_dir = 1;
  if (Left_Right_flag == 1 && vel_pos.y() < back_vel_pos.y())
    head_dir = 1;
  else if (Left_Right_flag == 1 && vel_pos.y() > back_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && vel_pos.y() < back_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && vel_pos.y() > back_vel_pos.y())
    head_dir = 1;
  MoveYDirection(vel_pos, back_vel_pos, forward_log_data, start_segment,
                 head_dir);
  // get point
  // combine forward_log_data and backward_log_data
  int icount = 0;
  // get point
  int forward_log_data_size = forward_log_data.size();
  for (int i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
    icount++;
  }
  int pp_planning_trajectory_size = pp_planning_trajectory.size();
  int backward_log_data_size = backward_log_data.size();
  int temp_segment =
      pp_planning_trajectory[pp_planning_trajectory_size - 1].segment_index();
  int temp_segment1 =
      backward_log_data[backward_log_data_size - 1].segment_index();
  int tmp_segment_index = temp_segment1;
  for (int i = backward_log_data_size - 1; i >= 0; i--) {
    pp_planning_trajectory.push_back(backward_log_data[i]);
    tmp_segment_index = temp_segment + temp_segment1 -
                        pp_planning_trajectory[icount].segment_index() + 1;
    pp_planning_trajectory[icount].set_segment_index(tmp_segment_index);
    if (pp_planning_trajectory[icount].direction() == 0)
      pp_planning_trajectory[icount].set_direction(1);
    else if (pp_planning_trajectory[icount].segment_index() == 1)
      pp_planning_trajectory[icount].set_direction(0);
    icount++;
  }
  pp_planning_trajectory_size = pp_planning_trajectory.size();
  // final point
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_direction(0);
  tmp_segment_index =
      pp_planning_trajectory[pp_planning_trajectory_size - 2].segment_index();
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_segment_index(
      tmp_segment_index);

  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}

bool PPP_ParkingIn::PerpendicularParkingNormalHeadIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: perpendicular to the carport*/
  ClassicTailInStep1(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /**********************************************
  Generate from carport origin
  ***********************************************/
  std::vector<TrajectoryPoint> backward_log_data;
  TrajectoryPoint back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  bool bflag = false;
  bflag = ppp_parking_out.Generate(back_vel_pos, backward_log_data, 2);
  if (bflag == false) return false;
  /*step5: calculate x-direction move or not*/
  // check whether x-direction OK?
  double new_final_x = backward_log_data[backward_log_data.size() - 1].x();
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];

  int head_dir = 1;
  if (Left_Right_flag == 2) head_dir = 2;
  // check x shift or not?
  if (back_vel_pos.x() > new_final_x + 0.2) {  // shift left
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
  } else if (back_vel_pos.x() < new_final_x - 0.2) {  // shift right
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(back_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x, forward_log_data,
                            start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return false;
  /*step 6: check y-direction move or not*/
  // check y shift or not?
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  back_vel_pos = backward_log_data[backward_log_data.size() - 1];
  head_dir = 1;
  if (Left_Right_flag == 1 && vel_pos.y() < back_vel_pos.y())
    head_dir = 1;
  else if (Left_Right_flag == 1 && vel_pos.y() > back_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && vel_pos.y() < back_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && vel_pos.y() > back_vel_pos.y())
    head_dir = 1;
  MoveYDirection(vel_pos, back_vel_pos, forward_log_data, start_segment,
                 head_dir);

  // get point
  // combine forward_log_data and backward_log_data
  int icount = 0;
  // get point
  int forward_log_data_size = forward_log_data.size();
  for (int i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
    icount++;
  }
  int pp_planning_trajectory_size = pp_planning_trajectory.size();
  int backward_log_data_size = backward_log_data.size();
  int temp_segment =
      pp_planning_trajectory[pp_planning_trajectory_size - 1].segment_index();
  int temp_segment1 =
      backward_log_data[backward_log_data_size - 1].segment_index();
  int tmp_segment_index = temp_segment1;
  for (int i = backward_log_data_size - 1; i >= 0; i--) {
    pp_planning_trajectory.push_back(backward_log_data[i]);
    tmp_segment_index = temp_segment + temp_segment1 -
                        pp_planning_trajectory[icount].segment_index() + 1;
    pp_planning_trajectory[icount].set_segment_index(tmp_segment_index);
    if (pp_planning_trajectory[icount].direction() == 0)
      pp_planning_trajectory[icount].set_direction(1);
    else if (pp_planning_trajectory[icount].direction() == 1)
      pp_planning_trajectory[icount].set_direction(0);
    icount++;
  }
  pp_planning_trajectory_size = pp_planning_trajectory.size();
  // final point
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_direction(0);
  tmp_segment_index =
      pp_planning_trajectory[pp_planning_trajectory_size - 2].segment_index();
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_segment_index(
      tmp_segment_index);

  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}

bool PPP_ParkingIn::PerpendicularParkingClassicHeadIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: perpendicular to the carport*/
  ClassicTailInStep1(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step2: adjust vehicle to constant position*/
  // xdirection [3.0~3.5m] to the forward road
  // ydirection [-0.2~0.2m] to the forward carport
  ClassicHeadInStep2(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step3:turn-forward to a certain angle*/
  ClassicHeadInStep3(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step4:Shift to x-axis*/
  ClassicHeadInStep4(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step5:go back to carport(0,0,0)*/
  ClassicHeadInStep5(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  // get point
  int forward_log_data_size = forward_log_data.size();

  for (int i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
void PPP_ParkingIn::ClassicHeadInStep2(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  TrajectoryPoint curr_vel_pos(forward_log_data[forward_log_data.size() - 1]);
  double final_x_val = curr_vel_pos.x();
  double new_final_x_val = 0.0;
  /*step2: adjust vehicle to constant position*/
  // xdirection [3.0~3.5m] to the forward road
  // ydirection [-0.2~0.2m] to the forward carport
  double xdistoforwardroad =
      final_x_val - GetMaxfromCarport('x', limit_road_front_coor_);
  xdistoforwardroad -= 1.2;
  if ((xdistoforwardroad <= 5.5) && (xdistoforwardroad >= 3.0))  // no change
    new_final_x_val = final_x_val;
  else
    new_final_x_val =
        GetMaxfromCarport('x', limit_road_front_coor_) + 1.2 + 3.2;
  int head_dir = 1;
  if (Left_Right_flag == 2) head_dir = 2;
  // check y shift or not?
  if (final_x_val > new_final_x_val + 0.2) {  // shift left
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
  } else if (final_x_val < new_final_x_val - 0.2) {  // shift right
    if (Left_Right_flag == 2)
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
    else
      ShiftXDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2,
                            head_dir, new_final_x_val, forward_log_data,
                            start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return;

  // check y shift or not?
  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = curr_vel_pos.x();
  tmp_y = -left_radius - 0.4;
  tmp_theta = M_PI_2;
  if (Left_Right_flag == 2) {
    tmp_y = right_radius + 0.2;
    tmp_theta = -M_PI_2;
  }
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  head_dir = 1;
  if (Left_Right_flag == 1 && curr_vel_pos.y() < end_vel_pos.y())
    head_dir = 1;
  else if (Left_Right_flag == 1 && curr_vel_pos.y() > end_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && curr_vel_pos.y() < end_vel_pos.y())
    head_dir = 2;
  else if (Left_Right_flag == 2 && curr_vel_pos.y() > end_vel_pos.y())
    head_dir = 1;
  MoveYDirection(curr_vel_pos, end_vel_pos, forward_log_data, start_segment,
                 head_dir);
  return;
}
void PPP_ParkingIn::ClassicHeadInStep3(
    TrajectoryPoint& vel_coor, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*step3:turn-forward to a certain angle*/
  bool bboardlimit = false;
  abnomous_watcher = 0;
  vel_coor = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  int flag = 0, flag_dir = 1;  // flag_dir:1 forward;0:backward;
  double tmp_radius = 0.0;
  bool step_3_flag = true;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  while (step_3_flag) {
    vel_coor = forward_log_data[forward_log_data.size() - 1];
    // 1. right-forward
    if (Left_Right_flag == 1) {  // left-forward
      flag = -1;
      tmp_radius =
          fabs(vel_coor.y()) / (1 - cos(M_PI - fabs(vel_coor.theta())));
      tmp_radius = fmin(15.0, tmp_radius);
      tmp_radius = fmax(left_radius, tmp_radius);
      flag_dir = -1;
    } else if (Left_Right_flag == 2) {  // right-forward
      flag = 1;
      tmp_radius =
          fabs(vel_coor.y()) / (1 - cos(M_PI - fabs(vel_coor.theta())));
      tmp_radius = fmin(15.0, tmp_radius);
      tmp_radius = fmax(right_radius, tmp_radius);
      flag_dir = 1;
    }
    origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
    origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
    b_carpot_flag = false;
    // save data defiine
    vel_coor.set_kappa(-flag / tmp_radius);     // curvature
    vel_coor.set_direction(0);                  // forward
    vel_coor.set_segment_index(start_segment);  // segment index
    double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
           tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {
      // first half of segment, move y
      if (fabs(tmp_y) > tmp_radius * 0.5) {
        if (Left_Right_flag == 1) {
          tmp_y -= (flag_dir * PARK_PRECISION);
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        } else if (Left_Right_flag == 2) {
          tmp_y -= (flag_dir * PARK_PRECISION);
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        }
      } else {  // second half of segmemt,move x
        if (Left_Right_flag == 1) {
          tmp_x -= PARK_PRECISION;
          tmp_x = fmax(0, tmp_x);
          tmp_y =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        } else if (Left_Right_flag == 2) {
          tmp_x -= PARK_PRECISION;
          tmp_x = fmax(0, tmp_x);
          tmp_y =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        }
      }

      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        forward_log_data.pop_back();  // give up the previous point
        break;
      }
      if (SatisfyHeadinStep3Check()) {  // anoymous
        abnomous_watcher = 1;
        return;
      }
      if (fabs(fabs(vel_coor.theta()) - M_PI) <= 0.05) {
        b_carpot_flag = true;
        step_3_flag = false;
      }

      forward_log_data.push_back(vel_coor);
    }
    start_segment++;
    vel_coor = forward_log_data[forward_log_data.size() - 1];
    if (step_3_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    // not finished?
    // 2. straight-backward
    double temp_k = tan(vel_coor.theta());
    double temp_b = vel_coor.y() - temp_k * vel_coor.x();
    double delta_x = 0.0;
    b_carpot_flag = false;
    vel_coor.set_kappa(0);                      // curvature
    vel_coor.set_direction(1);                  // forward
    vel_coor.set_segment_index(start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {  // insert straight line
      tmp_x += PARK_PRECISION;
      tmp_y = temp_k * tmp_x + temp_b;
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) {
        forward_log_data.pop_back();
        break;
      }
      // save data
      forward_log_data.push_back(vel_coor);
      delta_x += PARK_PRECISION;
      if (delta_x >= 0.7) break;
    }
    start_segment++;
    vel_coor = forward_log_data[forward_log_data.size() - 1];
    if (step_3_flag == false) break;
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 1) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}
// turn&shift to x-axis
void PPP_ParkingIn::ClassicHeadInStep4(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (vel_pos.x() > 7.0) {  // whether vehicle in carport
    abnomous_watcher = 1;
    return;
  }
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool bboardlimit = false;
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  if (fabs(vel_pos.y()) <= PARK_PRECISION) return;
  // need to shift y
  //(2):go backward to a certain position, 1.0m
  double shift_y = vel_pos.y();
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = vel_pos.x() + 1.0;
  tmp_y = vel_pos.y();
  tmp_theta = -M_PI + 0.01;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int head_dir = 1;
  if (vel_pos.x() < end_vel_pos.x())
    head_dir = 1;
  else
    head_dir = 2;
  MoveXDirectionHeadIn(vel_pos, end_vel_pos, forward_log_data, start_segment,
                       head_dir);
  //(3)backward, turn
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  double x_range = 0.0;
  x_range = GetMinfromCarport('x', limit_road_oppsite_coor_) -
            GetMinfromCarport('x', limit_carport_coor_);
  x_range -= (GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) -
              GetMinfromCarport('x', vehicle_pos_.rect.vertex_));
  if (x_range <= 1.0) {
    abnomous_watcher = 1;
    return;
  }
  x_range -= 0.8;
  if (shift_y > 0) {  // shift down
    ShiftYDirectionHeadIn(vel_pos, left_radius, right_radius, 1,
                          Left_Right_flag, 1, 0, x_range, forward_log_data,
                          start_segment, abnomous_watcher);
  } else {  // shift up
    ShiftYDirectionHeadIn(vel_pos, left_radius, right_radius, 2,
                          Left_Right_flag, 1, 0, x_range, forward_log_data,
                          start_segment, abnomous_watcher);
  }
  if (abnomous_watcher == 1) return;
  return;
}

// straight backward
void PPP_ParkingIn::ClassicHeadInStep5(
    TrajectoryPoint& vel_coor, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  vel_coor = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool bboardlimit = false;
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = VehicleParam::Instance()->left_front_y() + 0.2 +
          fmax(limit_carport_coor_[2].x(), limit_carport_coor_[3].x());
  tmp_y = vel_coor.y();
  tmp_theta = -M_PI;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int head_dir = 1;
  if (vel_coor.x() < end_vel_pos.x())
    head_dir = 1;
  else
    head_dir = 2;
  MoveXDirectionHeadIn(vel_coor, end_vel_pos, forward_log_data, start_segment,
                       head_dir);
  return;
}

bool PPP_ParkingIn::PerpendicularParkingShiftTailIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: parallel to the x-axis*/
  ShiftTailInStep1(vel_pos, forward_log_data, start_segment, abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step2:Shift to x-axis*/
  ClassicTailInStep4(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step3:go back to carport(0,0,0)*/
  ClassicTailInStep5(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  // get point
  int forward_log_data_size = forward_log_data.size();

  for (int i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
void PPP_ParkingIn::ShiftTailInStep1(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*
  step1: adjust vehicle parallel to x-axis
  */
  if (fabs(vel_pos.theta()) <= 0.11)  // already parallel
    return;
  double ver_dis_to_Up = 0.0, ver_dis_to_Down = 0.0;
  double temp_radius = 0.0;
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);

  ver_dis_to_Up = fabs(GetMaxfromCarport('y', limit_carport_coor_) -
                       GetMaxfromCarport('y', vehicle_pos_.rect.vertex_));
  ver_dis_to_Down = fabs(GetMinfromCarport('y', vehicle_pos_.rect.vertex_) -
                         GetMinfromCarport('y', limit_carport_coor_));
  temp_radius = left_radius;
  if (vel_pos.theta() > 0.01) {
    if (ver_dis_to_Up >= ver_dis_to_Down) {
      if (vel_pos.y() > 0 && ver_dis_to_Down > 0.5)  // left-backward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 1, 1,
                               forward_log_data, start_segment,
                               abnomous_watcher);
      else  // right-forward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 2, 0,
                               forward_log_data, start_segment,
                               abnomous_watcher);
    } else {
      if (vel_pos.y() < 0 && ver_dis_to_Up > 0.5)  // right-forward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 2, 0,
                               forward_log_data, start_segment,
                               abnomous_watcher);
      else  // left-backward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 1, 1,
                               forward_log_data, start_segment,
                               abnomous_watcher);
    }
  } else {
    if (ver_dis_to_Up >= ver_dis_to_Down) {
      if (vel_pos.y() > 0 && ver_dis_to_Down > 0.5)  // left-forward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 1, 0,
                               forward_log_data, start_segment,
                               abnomous_watcher);
      else  // right-backward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 2, 1,
                               forward_log_data, start_segment,
                               abnomous_watcher);
    } else {
      if (vel_pos.y() < 0 && ver_dis_to_Up > 0.5)  // right-backward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 2, 1,
                               forward_log_data, start_segment,
                               abnomous_watcher);
      else  // left-forward
        ShiftTailStep1Parallel(vel_pos, temp_radius, temp_radius, 1, 0,
                               forward_log_data, start_segment,
                               abnomous_watcher);
    }
  }
  return;
}
void PPP_ParkingIn::ShiftTailStep1Parallel(
    TrajectoryPoint vel_coor, double left_radius,
    double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
    int left_right_flag, int forward_back_flag,
    std::vector<TrajectoryPoint>&
        in_forward_log_data,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;log
                              // data
    int& in_start_segment,
    int& abnomous_watcher) {  // log and index; abnomous_watcher
  bool bboardlimit = false;
  abnomous_watcher = 0;
  vehicle_pos_.point = vel_coor;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  // begin to turn parallel
  int b_carpot_flag = 0;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  int flag = 0;
  double tmp_radius = 0.0, origin_x = 0.0, origin_y = 0.0;
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  while (!b_carpot_flag) {
    //(1)turn
    while (1) {
      if (left_right_flag == 2) {
        flag = -1;
        tmp_radius = right_radius;
      } else {
        flag = 1;
        tmp_radius = left_radius;
      }
      origin_x = -flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      if (forward_back_flag == 1)
        tmp_x -= PARK_PRECISION;
      else
        tmp_x += PARK_PRECISION;
      tmp_y =
          -flag * sqrt(tmp_radius * tmp_radius - pow((tmp_x - origin_x), 2)) +
          origin_y;
      tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
      RegulateYaw(tmp_theta);
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);

      vel_coor.set_kappa(flag / tmp_radius);
      if (forward_back_flag == 0)
        vel_coor.set_direction(0);
      else
        vel_coor.set_direction(1);
      vel_coor.set_segment_index(in_start_segment);
      in_forward_log_data.push_back(vel_coor);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        //	in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // check whether continue
      if (fabs(vel_coor.theta()) < 0.02) {
        b_carpot_flag = 1;
        break;
      }
    }
    if (b_carpot_flag == 1) break;
    in_start_segment++;
    i_abnormal_old_watcher = in_forward_log_data.size();
    //(2)change manuaver,continue
    while (1) {
      if (left_right_flag == 2) {
        flag = 1;
        tmp_radius = left_radius;
      } else {
        flag = -1;
        tmp_radius = right_radius;
      }
      origin_x = -flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      if (forward_back_flag == 1)
        tmp_x += PARK_PRECISION;
      else
        tmp_x -= PARK_PRECISION;
      tmp_y =
          -flag * sqrt(tmp_radius * tmp_radius - pow((tmp_x - origin_x), 2)) +
          origin_y;
      tmp_theta = atan2((origin_x - tmp_x), (tmp_y - origin_y));
      RegulateYaw(tmp_theta);
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      vel_coor.set_kappa(flag / tmp_radius);
      if (forward_back_flag == 0)
        vel_coor.set_direction(1);
      else
        vel_coor.set_direction(0);
      vel_coor.set_segment_index(in_start_segment);
      in_forward_log_data.push_back(vel_coor);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        //	in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // check whether continue
      if (fabs(vel_coor.theta()) < 0.02) {
        b_carpot_flag = 1;
        break;
      }
    }
    if (b_carpot_flag == 1) break;
    in_start_segment++;
    i_abnormal_watcher = in_forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  in_start_segment++;
  return;
}
}  // namespace planning
}  // namespace neodrive
