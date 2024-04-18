#include "ParallelParking_in.h"

namespace neodrive {
namespace planning {
/*
This function generate trajectory for normal pp_tail_in.
Conditions:vehicle should be on the road
Input: (1)vehicle start_pos, (2)Left_Right_flag:1:vehicle-up and
carport-down;2:vehicle-down and carport-up Output:trajectory_point
[x,y,z,yaw,curv,dis,vel,accel,forward/backward,segment index]
*/
bool PP_ParkingIn::ParallelParkingNormalTailIn(
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
  /*step1: adjust vehicle parallel to x-axis*/
  TailInStep1(vel_pos, forward_log_data, start_segment, abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  double old_final_y_val = forward_log_data[forward_log_data.size() - 1].y();
  /**********************************************
  Generate from carport origin
  ***********************************************/
  std::vector<TrajectoryPoint> backward_log_data;
  TrajectoryPoint back_vel_pos;
  // clear the init para
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  bool bflag = false;
  bflag = pp_parking_out.Generate(back_vel_pos, backward_log_data);
  if (bflag == false) {
    LOG_ERROR("ParallelParkingNormalTailIn use pp_parking_out failed");
    return false;
  }
  /*step8: calculate x-direction move or not*/
  // check whether y-direction OK?
  double new_final_y = backward_log_data[backward_log_data.size() - 1].y();
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (old_final_y_val > new_final_y + 0.2)  // y need move down
    ShiftYDirectionOnRoad(back_vel_pos, left_radius, right_radius, 2, 1,
                          new_final_y, forward_log_data, start_segment,
                          abnomous_watcher);
  else if (old_final_y_val < new_final_y - 0.2)  // y need move up
    ShiftYDirectionOnRoad(back_vel_pos, left_radius, right_radius, 1, 1,
                          new_final_y, forward_log_data, start_segment,
                          abnomous_watcher);
  if (abnomous_watcher == 1) {
    LOG_ERROR("ParallelParkingNormalTailIn ShiftYDirectionOnRoad failed.");
    return false;
  }
  // check x-direction move or not
  back_vel_pos = forward_log_data[forward_log_data.size() - 1];
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = backward_log_data[backward_log_data.size() - 1].x();
  tmp_y = backward_log_data[backward_log_data.size() - 1].y();
  tmp_theta = backward_log_data[backward_log_data.size() - 1].theta();
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int headtoend = 1;
  if (end_vel_pos.x() > back_vel_pos.x())
    headtoend = 1;
  else
    headtoend = 2;
  MoveXDirection(back_vel_pos, end_vel_pos, forward_log_data, start_segment,
                 headtoend);
  // combine forward_log_data and backward_log_data
  size_t icount = 0;
  // get point
  size_t forward_log_data_size = forward_log_data.size();
  for (size_t i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
    icount++;
  }
  int pp_planning_trajectory_size = pp_planning_trajectory.size();
  int backward_log_data_size = backward_log_data.size();
  int temp_segment =
      pp_planning_trajectory[pp_planning_trajectory_size - 1].segment_index();
  int temp_segment1 =
      backward_log_data[backward_log_data_size - 1].segment_index();
  int tmp_segment = 0;
  for (int i = backward_log_data_size - 1; i >= 0; i--) {
    pp_planning_trajectory.push_back(backward_log_data[i]);
    tmp_segment = temp_segment + temp_segment1 -
                  pp_planning_trajectory[icount].segment_index() + 1;

    pp_planning_trajectory[icount].set_segment_index(tmp_segment);
    if (pp_planning_trajectory[icount].direction() == 0)
      pp_planning_trajectory[icount].set_direction(1);
    else if (pp_planning_trajectory[icount].direction() == 1)
      pp_planning_trajectory[icount].set_direction(0);
    icount++;
  }
  pp_planning_trajectory_size = pp_planning_trajectory.size();
  // final point
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_direction(0);
  tmp_segment =
      pp_planning_trajectory[pp_planning_trajectory_size - 2].segment_index();
  pp_planning_trajectory[pp_planning_trajectory_size - 1].set_segment_index(
      tmp_segment);
  /*do not use, by chi*/
  /*std::reverse(pp_planning_trajectory.begin(), pp_planning_trajectory.end());
  for (int i = 0;i < pp_planning_trajectory.size();++i) {
  if (pp_planning_trajectory[i].direction_ == 0)
  pp_planning_trajectory[i].direction_ = 1;
  else
  pp_planning_trajectory[i].direction_ = 0;
  }*/
  /**/
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
void PP_ParkingIn::TailInStep1(TrajectoryPoint& vel_pos,
                               std::vector<TrajectoryPoint>& forward_log_data,
                               int& start_segment, int& abnomous_watcher) {
  /*
  step1: adjust vehicle parallel to x-axis
  */
  if (fabs(vel_pos.theta()) <= 0.11)  // already parallel
    return;
  // need to be parallel
  if (Left_Right_flag == 2) {  // vehicle up;carport down
    double ver_dis_to_carportC =
        fabs(vel_pos.y() - GetMaxfromCarport('y', limit_road_front_coor_));
    double ver_dis_to_roadLimit =
        fabs(GetMinfromCarport('y', limit_road_oppsite_coor_) - vel_pos.y());
    double temp_radius = 0.0;
    if (ver_dis_to_carportC >
        ver_dis_to_roadLimit) {  // close to road, need to be away from road
      temp_radius = left_radius;
      if (vel_pos.y() - temp_radius * (1 - cos(vel_pos.theta())) -
              protected_half_width - 1 >=
          GetMaxfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(
            fabs(vel_pos.y() - GetMaxfromCarport('y', limit_carport_coor_) -
                 protected_half_width - 1) /
                (1 - cos(vel_pos.theta())),
            temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // left-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // left-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
    } else {  // close to carport, need to be away from carport
      temp_radius = right_radius;
      if (vel_pos.y() + temp_radius * (1 - cos(vel_pos.theta())) -
              protected_half_width - 0.5 <=
          GetMaxfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(fabs(GetMaxfromCarport('y', limit_carport_coor_) +
                                protected_half_width + 0.5 - vel_pos.y()) /
                               (1 - cos(vel_pos.theta())),
                           temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // right-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // right-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
    }
  } else if (Left_Right_flag == 1) {  // vehicle down;carport up
    double ver_dis_to_carportC =
        fabs(GetMinfromCarport('y', limit_road_front_coor_) - vel_pos.y());
    double ver_dis_to_roadLimit =
        fabs(vel_pos.y() - GetMaxfromCarport('y', limit_road_oppsite_coor_));
    double temp_radius = 0.0;
    if (ver_dis_to_carportC >
        ver_dis_to_roadLimit) {  // close to road, need to be away from road
      temp_radius = right_radius;
      if (vel_pos.y() - temp_radius * (1 - cos(vel_pos.theta())) +
              protected_half_width + 1 <=
          GetMinfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(
            fabs(vel_pos.y() - GetMinfromCarport('y', limit_carport_coor_) +
                 protected_half_width + 1) /
                (1 - cos(vel_pos.theta())),
            temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // right-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // right-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
    } else {  // close to carport, need to be away from carport
      temp_radius = left_radius;
      if (vel_pos.y() + temp_radius * (1 - cos(vel_pos.theta())) +
              protected_half_width + 0.5 >=
          GetMinfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(fabs(GetMinfromCarport('y', limit_carport_coor_) -
                                protected_half_width - 0.5 - vel_pos.y()) /
                               (1 - cos(vel_pos.theta())),
                           temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // left-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // left-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
    }
  }
  return;
}

/*
This function make the vehicle parallel to the carport.
*/
// left_right_flag:1:left;2:right;forward_back_flag:0:forward;1:backward;
void PP_ParkingIn::ParallelParkingStep1RoadCarportCheck(
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
      origin_x = -flag * tmp_radius * sin(tmp_theta) + tmp_x;
      origin_y = flag * tmp_radius * cos(tmp_theta) + tmp_y;
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
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    while (1) {
      if (left_right_flag == 2) {
        flag = 1;
        tmp_radius = left_radius;
      } else {
        flag = -1;
        tmp_radius = right_radius;
      }
      origin_x = -flag * tmp_radius * sin(tmp_theta) + tmp_x;
      origin_y = flag * tmp_radius * cos(tmp_theta) + tmp_y;
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
// classical parallel parking tail in method
bool PP_ParkingIn::ParallelParkingClassicTailIn(
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
  /*step1: parallel to the carport*/
  TailInStep1(vel_pos, forward_log_data, start_segment, abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step2: adjust vehicle to constant position*/
  // ydirection [0.3~0.6m] to the forward road up
  // xdirection [-0.2~0.2m] to the forward road left
  ClassicTailInStep2(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step3:turn-back to a certain angle*/
  ClassicTailInStep3(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step4:straight backward*/
  ClassicTailInStep4(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step5:turn-back parallel to carport*/
  ClassicTailInStep5(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step6:ajust y to 0*/
  ClassicTailInStep6(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step7: ajust x to 0*/
  ClassicTailInStep7(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  // get point
  int forward_log_data_size = forward_log_data.size();
  for (int i = 0; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
    SaveDatatoBehavior(candidate_pos);
  }
  LogParkingData();
  return true;
}
void PP_ParkingIn::ClassicTailInStep2(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  TrajectoryPoint curr_vel_pos(forward_log_data[forward_log_data.size() - 1]);
  double final_y_val = curr_vel_pos.y();
  double new_final_y_val = 0.0;
  /*step2: adjust vehicle to constant position*/
  // ydirection [0.3~0.6m] to the forward road up
  // xdirection [-0.2~0.2m] to the forward road left
  if (Left_Right_flag == 2) {  // vehicle up;carport down
    double ydistoforwardroad =
        final_y_val - GetMaxfromCarport('y', limit_road_front_coor_);
    ydistoforwardroad -= protected_half_width;
    if ((ydistoforwardroad <= 0.6) && (ydistoforwardroad >= 0.3))  // no change
      new_final_y_val = final_y_val;
    else
      new_final_y_val = GetMaxfromCarport('y', limit_road_front_coor_) +
                        protected_half_width + 0.4;
    // check y shift or not?
    if (final_y_val > new_final_y_val + 0.2) {  // shift down
      ShiftYDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2, 1,
                            new_final_y_val, forward_log_data, start_segment,
                            abnomous_watcher);
    } else if (final_y_val < new_final_y_val - 0.2) {  // shift up
      ShiftYDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1, 1,
                            new_final_y_val, forward_log_data, start_segment,
                            abnomous_watcher);
    }
    if (abnomous_watcher == 1) return;
  } else if (Left_Right_flag == 1) {
    double ydistoforwardroad =
        GetMinfromCarport('y', limit_road_front_coor_) - final_y_val;
    ydistoforwardroad -= protected_half_width;
    if ((ydistoforwardroad <= 0.6) && (ydistoforwardroad >= 0.3))  // no change
      new_final_y_val = final_y_val;
    else
      new_final_y_val = GetMinfromCarport('y', limit_road_front_coor_) -
                        protected_half_width - 0.4;
    // check y shift or not?
    if (final_y_val > new_final_y_val + 0.2) {  // shift down
      ShiftYDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 2, 1,
                            new_final_y_val, forward_log_data, start_segment,
                            abnomous_watcher);
    } else if (final_y_val < new_final_y_val - 0.2) {  // shift up
      ShiftYDirectionOnRoad(curr_vel_pos, left_radius, right_radius, 1, 1,
                            new_final_y_val, forward_log_data, start_segment,
                            abnomous_watcher);
    }
    if (abnomous_watcher == 1) return;
  }
  // check x shift or not?
  curr_vel_pos = forward_log_data[forward_log_data.size() - 1];
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = limit_road_front_coor_[0].x();
  tmp_y = curr_vel_pos.y();
  tmp_theta = 0.0;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int headtoend = 1;
  if (end_vel_pos.x() > curr_vel_pos.x())
    headtoend = 1;
  else
    headtoend = 2;
  MoveXDirection(curr_vel_pos, end_vel_pos, forward_log_data, start_segment,
                 headtoend);
  return;
}
void PP_ParkingIn::ClassicTailInStep3(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  // calculate angle
  double limit_angle = 40.0 * M_PI / 180.0;
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  CalculateCalssicStep3Angle(vel_pos, limit_angle);
  // turn&back
  double tmp_radius = 0.0;
  int flag = 1;
  double origin_x = 0.0, origin_y = 0.0;
  bool step_3_flag = true;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  while (step_3_flag) {
    // 3.1 turn back
    if (Left_Right_flag == 1) {
      tmp_radius = left_radius;
      flag = 1;
    } else {
      tmp_radius = right_radius;
      flag = -1;
    }
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    origin_x = -flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
    origin_y = flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
    // turn
    bool b_carpot_flag = false;
    // save data define
    vel_pos.set_kappa(flag / tmp_radius);      // curvature
    vel_pos.set_direction(1);                  // backward
    vel_pos.set_segment_index(start_segment);  // segment index
    double tmp_x = vel_pos.x(), tmp_y = vel_pos.y(),
           tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      tmp_x -= PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) {
        forward_log_data.pop_back();
        break;
      }
      // save data
      forward_log_data.push_back(vel_pos);
      // check whether continue
      if (fabs(tmp_theta) >= fabs(limit_angle)) {
        b_carpot_flag = true;
        step_3_flag = false;
      }
    }
    start_segment++;
    if (step_3_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    /*step3.2 go backward*/
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    double temp_k = tan(vel_pos.theta());
    double temp_b = vel_pos.y() - temp_k * vel_pos.x();
    vel_pos.set_kappa(0);
    vel_pos.set_direction(1);
    vel_pos.set_segment_index(start_segment);

    b_carpot_flag = false;
    double delta_x = 0.0;
    tmp_x = vel_pos.x();
    tmp_y = vel_pos.y();
    tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      // insert straight line
      delta_x += PARK_PRECISION;
      tmp_x -= PARK_PRECISION;
      tmp_y = temp_k * tmp_x + temp_b;
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag == true) break;
      // save data
      forward_log_data.push_back(vel_pos);
      if (delta_x >= 0.5) b_carpot_flag = true;
    }
    start_segment++;
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}
// straight backward
void PP_ParkingIn::ClassicTailInStep4(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  bool b_carpot_flag = false;
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  double temp_k = tan(vel_pos.theta());
  double temp_b = vel_pos.y() - temp_k * vel_pos.x();
  vel_pos.set_kappa(0);
  vel_pos.set_direction(1);
  vel_pos.set_segment_index(start_segment);
  double tmp_x = vel_pos.x(), tmp_y = vel_pos.y(), tmp_theta = vel_pos.theta();
  while (!b_carpot_flag) {
    // insert straight line
    tmp_x -= PARK_PRECISION;
    tmp_y = temp_k * tmp_x + temp_b;
    vel_pos.set_x(tmp_x);
    vel_pos.set_y(tmp_y);
    vehicle_pos_.point = vel_pos;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    b_carpot_flag = CarportCurrBoundSafety();
    if (b_carpot_flag) {
      forward_log_data.pop_back();  // give up the previous point
      abnomous_watcher = 1;
      break;
    }
    // save data
    forward_log_data.push_back(vel_pos);
    // check condition
    if (GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) <
        (limit_road_front_coor_[0].x() - 2 * PARK_PRECISION))
      b_carpot_flag = true;
  }
  start_segment++;
  return;
}
// turn&back to parallel
void PP_ParkingIn::ClassicTailInStep5(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*This is add for shift in*/
  int abnor_flag = 0;
  if (Left_Right_flag == 2 && vel_pos.theta() < -0.05) {
    Left_Right_flag = 1;
    abnor_flag = 1;
  } else if (Left_Right_flag == 1 && vel_pos.theta() > 0.05) {
    Left_Right_flag = 2;
    abnor_flag = 1;
  }
  /*end*/
  // turn&back
  double tmp_radius = 0.0;
  int flag = 1;
  double origin_x = 0.0, origin_y = 0.0;
  if (Left_Right_flag == 2) {
    tmp_radius = left_radius;
    flag = 1;
  } else {
    tmp_radius = right_radius;
    flag = -1;
  }

  vel_pos = forward_log_data[forward_log_data.size() - 1];
  origin_x = -flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
  origin_y = flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
  // turn
  bool b_carpot_flag = false;
  vel_pos.set_kappa(flag / tmp_radius);      // curvature
  vel_pos.set_direction(1);                  // backward
  vel_pos.set_segment_index(start_segment);  // segment index
  double tmp_x = vel_pos.x(), tmp_y = vel_pos.y(), tmp_theta = vel_pos.theta();
  while (!b_carpot_flag) {
    tmp_x -= PARK_PRECISION;
    tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
    tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
    RegulateYaw(tmp_theta);
    vel_pos.set_x(tmp_x);
    vel_pos.set_y(tmp_y);
    vel_pos.set_theta(tmp_theta);
    vehicle_pos_.point = vel_pos;
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    b_carpot_flag = CarportCurrBoundSafety();
    if (b_carpot_flag) {
      forward_log_data.pop_back();  // give up the previous point
      break;
    }
    // save data
    forward_log_data.push_back(vel_pos);
    // check whether continue
    if (fabs(tmp_theta) <= 0.02) {
      b_carpot_flag = true;
    }
  }
  start_segment++;
  // check parallel or not?
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (fabs(tmp_theta) <= 0.02) return;
  // forward turn to parallel
  bool step_5_flag = true;
  int i_abnormal_old_watcher = 0, i_abnormal_watcher = 0;

  while (step_5_flag) {
    if (Left_Right_flag == 2) {
      tmp_radius = right_radius;
      flag = -1;
    } else {
      tmp_radius = left_radius;
      flag = 1;
    }
    origin_x = -flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
    origin_y = flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
    // turn
    b_carpot_flag = false;
    vel_pos.set_kappa(flag / tmp_radius);      // curvature
    vel_pos.set_direction(0);                  // backward
    vel_pos.set_segment_index(start_segment);  // segment index
    tmp_x = vel_pos.x();
    tmp_y = vel_pos.y();
    tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      tmp_x += PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) {
        forward_log_data.pop_back();  // give up the previous point
        break;
      }
      // save data
      forward_log_data.push_back(vel_pos);
      // check whether continue
      if (fabs(tmp_theta) <= 0.02) {
        step_5_flag = false;
        break;
      }
    }
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    start_segment++;
    if (step_5_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    /*
    oppisite turn
    */
    if (Left_Right_flag == 2) {
      tmp_radius = left_radius;
      flag = 1;
    } else {
      tmp_radius = right_radius;
      flag = -1;
    }
    origin_x = -flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
    origin_y = flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
    // turn
    b_carpot_flag = false;
    vel_pos.set_kappa(flag / tmp_radius);      // curvature
    vel_pos.set_direction(1);                  // backward
    vel_pos.set_segment_index(start_segment);  // segment index
    tmp_x = vel_pos.x();
    tmp_y = vel_pos.y();
    tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      tmp_x -= PARK_PRECISION;
      tmp_y = -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) {
        forward_log_data.pop_back();  // give up the previous point
        break;
      }
      // save data
      forward_log_data.push_back(vel_pos);
      // check whether continue
      if (fabs(tmp_theta) <= 0.02) {
        step_5_flag = false;
        break;
      }
    }
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    start_segment++;
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  /*This is add for shift in*/
  if (abnor_flag = 1 && Left_Right_flag == 2) {
    Left_Right_flag = 1;
  } else if (abnor_flag = 1 && Left_Right_flag == 1) {
    Left_Right_flag = 2;
  }
  /*end*/
  return;
}
// adjust y to 0
void PP_ParkingIn::ClassicTailInStep6(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  //(1):check y-ok or not
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (fabs(vel_pos.y()) <= PARK_PRECISION) return;
  //(2):go forward to a certain position, 0.4m to forward limit
  double shift_y = vel_pos.y();
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  double distofront = 0.0;
  distofront = GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) - vel_pos.x();
  TrajectoryPoint end_vel_pos;
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_x = limit_road_front_coor_[0].x() - PARK_PRECISION - distofront;
  tmp_y = vel_pos.y();
  tmp_theta = 0.0;
  end_vel_pos.set_x(tmp_x);
  end_vel_pos.set_y(tmp_y);
  end_vel_pos.set_theta(tmp_theta);
  int headtoend = 1;
  if (end_vel_pos.x() > vel_pos.x())
    headtoend = 1;
  else
    headtoend = 2;
  MoveXDirection(vel_pos, end_vel_pos, forward_log_data, start_segment,
                 headtoend);
  //(3)backward, turn
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  double x_range = 0.0;
  x_range = fmin(limit_carport_coor_[1].x(), limit_carport_coor_[2].x()) -
            fmax(limit_carport_coor_[0].x(), limit_carport_coor_[3].x());
  x_range -= (GetMaxfromCarport('x', vehicle_pos_.rect.vertex_) -
              GetMinfromCarport('x', vehicle_pos_.rect.vertex_));
  if (x_range <= 1.0) {
    abnomous_watcher = 1;
    return;
  }
  x_range -= 0.4;
  if (Left_Right_flag == 2 && shift_y > 0)  // vehicle up,shift down
    ShiftYDirection(vel_pos, left_radius, right_radius, 2, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  else if (Left_Right_flag == 2 && shift_y < 0)  // vehicle up,shift up
    ShiftYDirection(vel_pos, left_radius, right_radius, 1, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  else if (Left_Right_flag == 1 && shift_y > 0)  // vehicle down,shift down
    ShiftYDirection(vel_pos, left_radius, right_radius, 2, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  else if (Left_Right_flag == 1 && shift_y < 0)  // vehicle down,shift up
    ShiftYDirection(vel_pos, left_radius, right_radius, 1, 1, 0, x_range,
                    forward_log_data, start_segment, abnomous_watcher);
  if (abnomous_watcher == 1) return;
  return;
}
void PP_ParkingIn::ClassicTailInStep7(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  //(1):check x-ok or not
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  if (fabs(vel_pos.x()) < PARK_PRECISION) return;
  //(2):forward or backward
  TrajectoryPoint end_vel_pos;
  end_vel_pos.set_y(vel_pos.y());
  int headtoend = 1;
  if (end_vel_pos.x() > vel_pos.x())
    headtoend = 1;
  else
    headtoend = 2;
  if (vel_pos.x() > 0) {  // backward
    MoveXDirection(vel_pos, end_vel_pos, forward_log_data, start_segment,
                   headtoend);
  } else if (vel_pos.x() < 0) {  // forward
    MoveXDirection(vel_pos, end_vel_pos, forward_log_data, start_segment,
                   headtoend);
  }
  return;
}
void PP_ParkingIn::CalculateCalssicStep3Angle(TrajectoryPoint& vel_pos,
                                              double& angle) {
  double x_limit = 0.0, y_limit = 0.0;
  double target_angle = 0.0;
  if (Left_Right_flag == 2) {
    x_limit = limit_carport_coor_[3].x();
    y_limit = limit_carport_coor_[3].y();
    target_angle = 1.1 * atan2(vel_pos.y() - y_limit, vel_pos.x() - x_limit);
    RegulateYaw(target_angle);
    angle = target_angle;
  } else if (Left_Right_flag == 1) {
    x_limit = limit_carport_coor_[3].x();
    y_limit = limit_carport_coor_[3].y();
    target_angle = 1.1 * atan2(vel_pos.y() - y_limit, vel_pos.x() - x_limit);
    RegulateYaw(target_angle);
    angle = target_angle;
  }
  return;
}
bool PP_ParkingIn::ParallelParkingShiftIn(
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
  /*step4:straight backward*/
  ClassicTailInStep4(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step5:turn-back parallel to carport*/
  ClassicTailInStep5(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step2:ajust y to 0*/
  ClassicTailInStep6(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  /*step7: ajust x to 0*/
  ClassicTailInStep7(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (abnomous_watcher == 1) return false;
  // get point
  int forward_log_data_size = forward_log_data.size();
  for (int i = 1; i < forward_log_data_size; ++i) {
    pp_planning_trajectory.push_back(forward_log_data[i]);
  }
  SaveDatatoBehavior(candidate_pos);
  LogParkingData();
  return true;
}
void PP_ParkingIn::ShiftInStep1(TrajectoryPoint& vel_pos,
                                std::vector<TrajectoryPoint>& forward_log_data,
                                int& start_segment, int& abnomous_watcher) {
  /*
  step1: adjust vehicle parallel to x-axis
  */
  if (fabs(vel_pos.theta()) <= 0.11)  // already parallel
    return;
  // need to be parallel
  if (Left_Right_flag == 2) {  // vehicle up;carport down
    double ver_dis_to_carportC =
        fabs(vel_pos.y() - GetMaxfromCarport('y', limit_road_front_coor_));
    double ver_dis_to_roadLimit =
        fabs(GetMinfromCarport('y', limit_road_oppsite_coor_) - vel_pos.y());
    double temp_radius = 0.0;
    if (ver_dis_to_carportC >
        ver_dis_to_roadLimit) {  // close to road, need to be away from road
      temp_radius = left_radius;
      if (vel_pos.y() - temp_radius * (1 - cos(vel_pos.theta())) -
              protected_half_width - 1 >=
          GetMaxfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(
            fabs(vel_pos.y() - GetMaxfromCarport('y', limit_carport_coor_) -
                 protected_half_width - 1) /
                (1 - cos(vel_pos.theta())),
            temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // left-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // left-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
    } else {  // close to carport, need to be away from carport
      temp_radius = right_radius;
      if (vel_pos.y() + temp_radius * (1 - cos(vel_pos.theta())) -
              protected_half_width - 0.5 <=
          GetMaxfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(fabs(GetMaxfromCarport('y', limit_carport_coor_) +
                                protected_half_width + 0.5 - vel_pos.y()) /
                               (1 - cos(vel_pos.theta())),
                           temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // right-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // right-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
    }
  } else if (Left_Right_flag == 1) {  // vehicle down;carport up
    double ver_dis_to_carportC =
        fabs(GetMinfromCarport('y', limit_road_front_coor_) - vel_pos.y());
    double ver_dis_to_roadLimit =
        fabs(vel_pos.y() - GetMaxfromCarport('y', limit_road_oppsite_coor_));
    double temp_radius = 0.0;
    if (ver_dis_to_carportC >
        ver_dis_to_roadLimit) {  // close to road, need to be away from road
      temp_radius = right_radius;
      if (vel_pos.y() - temp_radius * (1 - cos(vel_pos.theta())) +
              protected_half_width + 1 <=
          GetMinfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(
            fabs(vel_pos.y() - GetMinfromCarport('y', limit_carport_coor_) +
                 protected_half_width + 1) /
                (1 - cos(vel_pos.theta())),
            temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // right-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // right-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             2, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
    } else {  // close to carport, need to be away from carport
      temp_radius = left_radius;
      if (vel_pos.y() + temp_radius * (1 - cos(vel_pos.theta())) +
              protected_half_width + 0.5 >=
          GetMinfromCarport('y', limit_carport_coor_)) {
        temp_radius = fmax(fabs(GetMinfromCarport('y', limit_carport_coor_) -
                                protected_half_width - 0.5 - vel_pos.y()) /
                               (1 - cos(vel_pos.theta())),
                           temp_radius);
      }
      if (vel_pos.theta() <= -0.1)  // left-forward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 0, forward_log_data,
                                             start_segment, abnomous_watcher);
      else if (vel_pos.theta() >= 0.1)  // left-backward
        ParallelParkingStep1RoadCarportCheck(vel_pos, temp_radius, temp_radius,
                                             1, 1, forward_log_data,
                                             start_segment, abnomous_watcher);
    }
  }
  return;
}

}  // namespace planning
}  // namespace neodrive
