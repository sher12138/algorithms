#include "AngularParking_in.h"
namespace neodrive {
namespace planning {

// TODO(wyc): reconstrut this file
bool AP_ParkingIn::AngularParkingNormalTailIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  // TODO(wyc): define variable before using
  // vector forward_log_data{vel_pos};
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos(start_pos);
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: perpendicular to the carport*/
  // ClassicTailInStep1(vel_pos, forward_log_data,
  //	start_segment, abnomous_watcher);
  // if (abnomous_watcher == 1)
  //	return false;
  /**********************************************
  Generate from carport origin
  ***********************************************/
  std::vector<TrajectoryPoint> backward_log_data;
  TrajectoryPoint back_vel_pos;
  // clear the init para
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  bool bflag = false;
  bflag = ap_parking_out.Generate(back_vel_pos, backward_log_data, 1);
  if (bflag == false) return false;
  /*step5: calculate x-direction move or not*/
  // check whether x-direction OK?

  double new_final_x = backward_log_data[backward_log_data.size() - 1].x();
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];

  int head_dir = 1;
  if (Left_Right_flag == 2) head_dir = 2;
  // check x shift or not?
  // TODO(wyc): bad code style
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
  ///*step 6: check y-direction move or not*/
  ////check y shift or not?
  // vel_pos = forward_log_data[forward_log_data.size() - 1];
  // back_vel_pos = backward_log_data[backward_log_data.size() - 1];
  // head_dir = 1;
  // if (Left_Right_flag == 1 && vel_pos.y_ < back_vel_pos.y_)
  //	head_dir = 1;
  // else if (Left_Right_flag == 1 && vel_pos.y_ > back_vel_pos.y_)
  //	head_dir = 2;
  // else if (Left_Right_flag == 2 && vel_pos.y_ < back_vel_pos.y_)
  //	head_dir = 2;
  // else if (Left_Right_flag == 2 && vel_pos.y_ > back_vel_pos.y_)
  //	head_dir = 1;
  // MoveYDirection(vel_pos, back_vel_pos, forward_log_data,
  //	start_segment, head_dir);
  // get point
  // combine forward_log_data and backward_log_data
  int icount = 0;
  // get point
  // TODO(wyc): confusing
  // TODO(wyc): auto pp_planning_trajectory = forward_log_data;
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

  pp_planning_trajectory_size = pp_planning_trajectory.size();
  candidate_pos.clear();
  for (int i = 0; i < pp_planning_trajectory_size; ++i) {
    candidate_pos.push_back(pp_planning_trajectory[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
bool AP_ParkingIn::AngularParkingClassicTailIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  return true;
}
bool AP_ParkingIn::AngularParkingNormalHeadIn(
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
  bflag = ap_parking_out.Generate(back_vel_pos, backward_log_data, 2);
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

  pp_planning_trajectory_size = pp_planning_trajectory.size();
  candidate_pos.clear();
  for (int i = 0; i < pp_planning_trajectory_size; ++i) {
    candidate_pos.push_back(pp_planning_trajectory[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
bool AP_ParkingIn::AngularParkingClassicHeadIn(
    TrajectoryPoint start_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  return true;
}
// perpendicular to the carport
void AP_ParkingIn::ClassicTailInStep1(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*step1: adjust vehicle parallel to y-axis*/
  double tmp_theta = vel_pos.theta();
  if (Left_Right_flag == 1)
    RegulateYaw0Pi(tmp_theta);
  else if (Left_Right_flag == 2)
    RegulateYawPi0(tmp_theta);
  vel_pos.set_theta(tmp_theta);
  if (fabs(fabs(tmp_theta) - M_PI_2) <= 0.05) {  // already parallel
    return;
  }
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
  double delta_yaw = fabs(M_PI_2 - fabs(vel_pos.theta()));
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
      step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 1, 0,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 2:  // left-backward
      step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 1, 1,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 3:  // right-backward
      step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 2, 1,
                           forward_log_data, start_segment, abnomous_watcher);
      break;
    case 4:  // right-forward
      step1ParalleltoYAxis(vel_pos, temp_radius, temp_radius, 2, 0,
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
void AP_ParkingIn::step1ParalleltoYAxis(
    TrajectoryPoint vel_coor, double left_radius,
    double right_radius,  // vel_coor:current pos; tmp_radius: turn radius
    int left_right_flag, int forward_back_flag,
    std::vector<TrajectoryPoint>&
        in_forward_log_data,  // left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;log
                              // data
    int& in_start_segment, int& abnomous_watcher) {  // log and index
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
}  // namespace planning
}  // namespace neodrive
