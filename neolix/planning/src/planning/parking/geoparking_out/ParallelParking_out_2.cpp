#include "ParallelParking_out.h"
/*
This function generate trajectory for normal pp_out.
Input:vehicle end_pos, only use y-value.
Output:trajectory_point.
[x,y,z,yaw,curv,dis,vel,accel,forward/backward,segment index]
*/
namespace neodrive {
namespace planning {

bool PP_ParkingOut::ParallelParkingNormalOut(
    TrajectoryPoint end_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos;
  vel_pos.Clear();
  vel_pos.set_segment_index(start_segment);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;
  /*step1: start from carport origin, go backward*/
  TailOutStep1(vel_pos, forward_log_data, start_segment);
  /*
  step2:turn, out, limit-->satisfy boundary
            turn, back,limit-->satisfy boundary
  */
  TailOutStep2(vel_pos, forward_log_data, start_segment, abnomous_watcher);
  if (abnomous_watcher == 1) {
    LOG_ERROR("TailOutStep2 failed");
    return false;
  }
  /*step3: check go forward or backward*/
  TailOutStep3(vel_pos, forward_log_data, start_segment, abnomous_watcher,
               end_pos.y());
  if (abnomous_watcher == 1) {
    LOG_ERROR("TailOutStep3 failed");
    return false;
  }
  /*step4: turn parallel*/
  TailOutStep4(vel_pos, forward_log_data, start_segment, abnomous_watcher,
               end_pos.y());
  if (abnomous_watcher == 1) {
    LOG_ERROR("TailOutStep4 failed");
    return false;
  }
  // get point
  size_t forward_log_data_size = forward_log_data.size();
  for (size_t i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}

void PP_ParkingOut::TailOutStep1(TrajectoryPoint& curr_vel_pos,
                                 std::vector<TrajectoryPoint>& forward_log_data,
                                 int& start_segment) {
  /*
  step3: start from carport origin, go backward
  */
  start_segment = 1;
  vehicle_pos_.point = curr_vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bool b_carpot_flag = CurrCarportBoundSafety(1);
  int icount = 0;
  // save data define
  curr_vel_pos.set_kappa(0);                      // curvature
  curr_vel_pos.set_segment_index(start_segment);  // segment index
  curr_vel_pos.set_direction(1);                  // backward

  double max_backdis = 2.5;
  double delta_x = 0.0;
  double tmp_x = curr_vel_pos.x();
  while (!b_carpot_flag) {
    tmp_x -= PARK_PRECISION;
    curr_vel_pos.set_x(tmp_x);
    vehicle_pos_.point = curr_vel_pos;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    b_carpot_flag = CurrCarportBoundSafety(curr_vel_pos.direction());
    // save data
    forward_log_data.push_back(curr_vel_pos);
    icount++;
    delta_x += PARK_PRECISION;
    if (delta_x >= max_backdis) b_carpot_flag = true;
  }
  // delete the last data
  if (icount >= 2) {
    forward_log_data.pop_back();
    forward_log_data.pop_back();
  } else if (icount == 1)
    forward_log_data.pop_back();

  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  start_segment++;
  return;
}
/*
step2.1:turn, out, limit-->satisfy boundary
step2.2:turn, back,limit-->satisfy boundary
*/
void PP_ParkingOut::TailOutStep2(TrajectoryPoint& curr_vel_pos,
                                 std::vector<TrajectoryPoint>& forward_log_data,
                                 int& start_segment, int& abnomous_watcher) {
  /*
  step2.1:turn, out, limit-->satisfy boundary
  step2.2:turn, back,limit-->satisfy boundary
  */
  bool step_2_flag = true;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  double tmp_radius = 0.0, origin_x = 0.0, origin_y = 0.0;
  int flag = 0;
  bool finish_flag = false;
  int saved_data_num = 0;  // used to pop out data once collision

  while (step_2_flag) {
    if (Left_Right_flag == 1) {
      tmp_radius = right_radius;
      flag = -1;
    } else {
      tmp_radius = left_radius;
      flag = 1;
    }
    curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
    origin_x =
        -flag * tmp_radius * sin(curr_vel_pos.theta()) + curr_vel_pos.x();
    origin_y = flag * tmp_radius * cos(curr_vel_pos.theta()) + curr_vel_pos.y();
    // turn
    bool b_carpot_flag = false;
    // save data define
    curr_vel_pos.set_kappa(flag / tmp_radius);      // curvature
    curr_vel_pos.set_direction(0);                  // forward
    curr_vel_pos.set_segment_index(start_segment);  // segment index
    double tmp_x = curr_vel_pos.x(), tmp_y = curr_vel_pos.y(),
           tmp_theta = curr_vel_pos.theta();
    while (!b_carpot_flag) {
      tmp_x += PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = curr_vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CurrCarportBoundSafety(curr_vel_pos.direction());
      if (b_carpot_flag) {
        if (saved_data_num >= 4) {
          forward_log_data.pop_back();
          //	forward_log_data.pop_back();
        } else if (saved_data_num >= 1) {
          forward_log_data.pop_back();
        }
        break;
      }
      // save data
      forward_log_data.push_back(curr_vel_pos);
      saved_data_num += 1;
      // check whether continue
      finish_flag = Satisfystep2Check();
      if (finish_flag == true) {
        step_2_flag = false;
        break;
      }
    }
    start_segment++;
    if (step_2_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    /*
    step2.2:turn, in, limit-->satisfy boundary
    */
    if (Left_Right_flag == 1) {
      tmp_radius = left_radius;
      flag = 1;
    } else {
      tmp_radius = right_radius;
      flag = -1;
    }
    curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
    tmp_x = curr_vel_pos.x();
    tmp_y = curr_vel_pos.y();
    tmp_theta = curr_vel_pos.theta();
    origin_x =
        -flag * tmp_radius * sin(curr_vel_pos.theta()) + curr_vel_pos.x();
    origin_y = flag * tmp_radius * cos(curr_vel_pos.theta()) + curr_vel_pos.y();
    saved_data_num = 0;
    // turn
    b_carpot_flag = false;
    // save data define
    curr_vel_pos.set_kappa(flag / tmp_radius);      // curvature
    curr_vel_pos.set_direction(1);                  // backward
    curr_vel_pos.set_segment_index(start_segment);  // segment index
    while (!b_carpot_flag) {
      tmp_x -= PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = curr_vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CurrCarportBoundSafety(curr_vel_pos.direction());
      if (b_carpot_flag) {
        if (saved_data_num >= 4) {
          forward_log_data.pop_back();
          //	forward_log_data.pop_back();
        } else if (saved_data_num >= 1) {
          forward_log_data.pop_back();
        }
        break;
      }
      // save data
      forward_log_data.push_back(curr_vel_pos);
      saved_data_num += 1;
      // check whether continue
      finish_flag = Satisfystep2Check();
      if (finish_flag == true) {
        step_2_flag = false;
        break;
      }
    }
    curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
    start_segment++;
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}
/*
step3: check go forward or backward
*/
void PP_ParkingOut::TailOutStep3(TrajectoryPoint& curr_vel_pos,
                                 std::vector<TrajectoryPoint>& forward_log_data,
                                 int& start_segment, int& abnomous_watcher,
                                 double final_y_val) {
  // calculate ideal longitudinal offset
  bool b_carpot_flag = false;
  double ideal_y_offset = 0.0;
  int forward_backward_flag = 0;
  double first_select_dis = -1;
  double temp_dis = 0.0;
  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (Left_Right_flag == 2) {  // vehicle up;carport down
    ideal_y_offset = right_radius * (1 - cos(curr_vel_pos.theta()));
    if ((curr_vel_pos.y() >
         final_y_val - ideal_y_offset + PARK_PRECISION)) {  // need backward
      forward_backward_flag = 1;
      first_select_dis = final_y_val - ideal_y_offset + PARK_PRECISION;
    } else if ((curr_vel_pos.y() < final_y_val - ideal_y_offset -
                                       PARK_PRECISION)) {  // need forward
      forward_backward_flag = 0;
      first_select_dis = final_y_val - ideal_y_offset - PARK_PRECISION;
      // collision with road
      temp_dis = GetMinfromCarport('y', limit_road_oppsite_coor_) +
                 right_radius -
                 sqrt(pow(right_radius + protected_half_width, 2) + 4.2 * 4.2) -
                 PARK_PRECISION;
      first_select_dis = fmin(first_select_dis, temp_dis);
    } else
      b_carpot_flag = true;
  } else if (Left_Right_flag == 1) {  // vehicle down;carport up
    ideal_y_offset = left_radius * (1 - cos(curr_vel_pos.theta()));
    if ((curr_vel_pos.y() <
         final_y_val + ideal_y_offset - PARK_PRECISION)) {  // need backward
      forward_backward_flag = 1;
      first_select_dis = final_y_val + ideal_y_offset - PARK_PRECISION;
    } else if ((curr_vel_pos.y() > final_y_val + ideal_y_offset +
                                       PARK_PRECISION)) {  // need forward
      forward_backward_flag = 0;
      first_select_dis = final_y_val + ideal_y_offset + PARK_PRECISION;
      temp_dis = GetMaxfromCarport('y', limit_road_oppsite_coor_) -
                 left_radius +
                 sqrt(pow(left_radius + protected_half_width, 2) + 4.2 * 4.2) +
                 PARK_PRECISION;
      first_select_dis = fmax(first_select_dis, temp_dis);
    } else
      b_carpot_flag = true;
  }
  // save data define
  double temp_k = tan(curr_vel_pos.theta());
  double temp_b = curr_vel_pos.y() - temp_k * curr_vel_pos.x();
  double delta_x = 0.0;
  if (forward_backward_flag == 0)
    delta_x = PARK_PRECISION;
  else if (forward_backward_flag == 1)
    delta_x = -PARK_PRECISION;
  curr_vel_pos.set_kappa(0);
  curr_vel_pos.set_direction(forward_backward_flag);
  curr_vel_pos.set_segment_index(start_segment);
  int saved_data_num = 0;  // used to pop out data once collision
  double tmp_x = curr_vel_pos.x(), tmp_y = curr_vel_pos.y();
  while (!b_carpot_flag) {  // insert straight line
    tmp_x += delta_x;
    tmp_y = temp_k * tmp_x + temp_b;
    curr_vel_pos.set_x(tmp_x);
    curr_vel_pos.set_y(tmp_y);
    vehicle_pos_.point = curr_vel_pos;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    b_carpot_flag = CarportCurrBoundSafety();
    if (b_carpot_flag) {
      if (saved_data_num >= 4) {
        forward_log_data.pop_back();
        //	forward_log_data.pop_back();
      } else if (saved_data_num >= 1) {
        forward_log_data.pop_back();
      }
      break;
    }
    // save data
    forward_log_data.push_back(curr_vel_pos);
    saved_data_num += 1;
    if (Left_Right_flag == 2) {
      if ((tmp_y >= first_select_dis) && (forward_backward_flag == 0))
        b_carpot_flag = true;
      else if ((tmp_y <= first_select_dis) && (forward_backward_flag == 1))
        b_carpot_flag = true;
    } else if (Left_Right_flag == 1) {
      if ((tmp_y <= first_select_dis) && (forward_backward_flag == 0))
        b_carpot_flag = true;
      else if ((tmp_y >= first_select_dis) && (forward_backward_flag == 1))
        b_carpot_flag = true;
    }
  }
  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  start_segment++;
  return;
}
void PP_ParkingOut::TailOutStep4(TrajectoryPoint& curr_vel_pos,
                                 std::vector<TrajectoryPoint>& forward_log_data,
                                 int& start_segment, int& abnomous_watcher,
                                 double final_y_val) {
  /*
  step4.1: turn parallel
  */
  bool step_4_flag = false;
  bool warningflag = false;  // go backward
  bool finish_flag = false;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  double delta_x = 0.0;
  double tmp_radius = 0.0, origin_x = 0.0, origin_y = 0.0;
  int flag = 0;
  while (!step_4_flag) {
    curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
    if (Left_Right_flag == 2) {
      // calculate turn radius
      tmp_radius =
          (final_y_val - curr_vel_pos.y()) / (1 - cos(curr_vel_pos.theta()));
      if (tmp_radius < 1 / GetLimitRightCurv())
        tmp_radius = 1 / GetLimitRightCurv();
      tmp_radius = fmin(15.0, tmp_radius);
      delta_x = tmp_radius * cos(curr_vel_pos.theta()) + curr_vel_pos.x();
      warningflag = false;
      flag = -1;
    } else if (Left_Right_flag == 1) {
      // calculate turn radius
      tmp_radius =
          (-final_y_val + curr_vel_pos.y()) / (1 - cos(curr_vel_pos.theta()));
      if (tmp_radius < 1 / GetLimitLeftCurv())
        tmp_radius = 1 / GetLimitLeftCurv();
      tmp_radius = fmin(15.0, tmp_radius);
      delta_x = tmp_radius * cos(curr_vel_pos.theta()) + curr_vel_pos.x();
      warningflag = false;
      flag = 1;
    }
    origin_x =
        -flag * tmp_radius * sin(curr_vel_pos.theta()) + curr_vel_pos.x();
    origin_y = flag * tmp_radius * cos(curr_vel_pos.theta()) + curr_vel_pos.y();
    // turn
    bool b_carpot_flag = false;
    // save data define
    curr_vel_pos.set_kappa(flag / tmp_radius);      // curvature
    curr_vel_pos.set_direction(0);                  // forward
    curr_vel_pos.set_segment_index(start_segment);  // segment index
    double tmp_x = curr_vel_pos.x(), tmp_y = curr_vel_pos.y(),
           tmp_theta = curr_vel_pos.theta();
    while (!b_carpot_flag) {
      tmp_x += PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = curr_vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag == true) warningflag = true;
      finish_flag = Satisfystep4Check();
      if (finish_flag == true) {
        abnomous_watcher = 1;
        return;
      }
      // save data
      forward_log_data.push_back(curr_vel_pos);
      // check whether to continue
      if (fabs(tmp_theta) < 0.02) {
        b_carpot_flag = true;
        step_4_flag = true;
      }
    }
    start_segment++;
    if (step_4_flag == true) break;
    i_abnormal_old_watcher = forward_log_data.size();
    // according to warning flag, go straight back, and turn again
    if (warningflag == true) {
      curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
      double temp_k = tan(tmp_theta);
      double temp_b = tmp_y - temp_k * tmp_x;
      curr_vel_pos.set_kappa(0);
      curr_vel_pos.set_direction(1);
      curr_vel_pos.set_segment_index(start_segment);

      b_carpot_flag = false;
      delta_x = 0.0;
      tmp_x = curr_vel_pos.x();
      tmp_y = curr_vel_pos.y();
      tmp_theta = curr_vel_pos.theta();
      while (!b_carpot_flag) {
        // insert straight line
        delta_x += PARK_PRECISION;
        tmp_x -= PARK_PRECISION;
        tmp_y = temp_k * tmp_x + temp_b;
        curr_vel_pos.set_x(tmp_x);
        curr_vel_pos.set_y(tmp_y);
        vehicle_pos_.point = curr_vel_pos;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        b_carpot_flag = CarportCurrBoundSafety();
        if (b_carpot_flag == true) break;
        // save data
        forward_log_data.push_back(curr_vel_pos);
        if (delta_x >= 0.7) b_carpot_flag = true;
      }
      start_segment++;
      i_abnormal_watcher = forward_log_data.size();
      if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
        abnomous_watcher = 1;
        return;
      }
    }
  }
  return;
}

}  // namespace planning
}  // namespace neodrive
