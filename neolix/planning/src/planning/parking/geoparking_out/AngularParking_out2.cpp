#include "AngularParking_out.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

bool AP_ParkingOut::AngularParkingNormalHeadOut(
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
  /*step1: start from carport origin, go forward*/
  NormalHeadOutStep1(vel_pos, forward_log_data, start_segment);
  /*step2: turn parallel to y-axis*/
  NormalHeadOutStep2(vel_pos, forward_log_data, start_segment, abnomous_watcher,
                     end_pos.x(), end_pos.theta());
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
// go forward
void AP_ParkingOut::NormalHeadOutStep1(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment) {
  /*step1.1: start from carport origin, check dis to oppsite dis*/
  double oppsite_dis = 0.0, corner_dis = 0.0;

  oppsite_dis =
      fabs(limit_carport_coor_[0].y() - vel_pos.y()) - protected_half_width;
  corner_dis =
      fabs(limit_carport_coor_[1].y() - vel_pos.y()) - protected_half_width;
  if (corner_dis <= 1.5 * protected_half_width)
    if (oppsite_dis > 0.5 * protected_half_width) {  // shift
      double shift_target = 0.0;
      if (Left_Right_flag == 1) {
        shift_target = limit_carport_coor_[0].y() + 1.5 * protected_half_width;
        ShiftYDirection(vel_pos, left_radius, right_radius, 2, 0, shift_target,
                        6.0, forward_log_data, start_segment);
      } else if (Left_Right_flag == 2) {
        shift_target = limit_carport_coor_[0].y() - 1.5 * protected_half_width;
        ShiftYDirection(vel_pos, left_radius, right_radius, 1, 0, shift_target,
                        6.0, forward_log_data, start_segment);
      }
    }
  if (forward_log_data.size() > 1)
    vel_pos = forward_log_data[forward_log_data.size() - 1];
  else
    vel_pos.Clear();
  /*step1.2: start from carport origin, go forward*/
  double target_x = 0.0;
  target_x = fmax(limit_carport_coor_[1].x(), limit_carport_coor_[0].x());
  // calculate side area
  double disto_corner = 0.0;
  disto_corner =
      fabs(limit_carport_coor_[1].y() - vel_pos.y()) - protected_half_width;
  if (disto_corner > 0.5) {
    target_x -= disto_corner * 0.8;
    target_x = fmax(0, target_x);
  }
  vel_pos.set_kappa(0);                      // curvature
  vel_pos.set_segment_index(start_segment);  // segment index
  vel_pos.set_direction(0);                  // forward
  double tmp_x = vel_pos.x();
  while (1) {
    tmp_x += PARK_PRECISION;
    if (tmp_x > target_x) break;
    // save data
    vel_pos.set_x(tmp_x);
    forward_log_data.push_back(vel_pos);
  }
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  start_segment++;
  return;
}
// turn&out
void AP_ParkingOut::NormalHeadOutStep2(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher, double final_x_val,
    double theta_final) {
  /*step2: turn out parallel to theta_final*/
  bool step_2_flag = false;
  bool warningflag = false;
  bool finish_flag = false;
  double delta_x = 0.0;
  double tmp_radius = 0.0, origin_x = 0.0, origin_y = 0.0;
  int flag = 0;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  int saved_data_num = 0;  // used to pop out data once collision
  while (!step_2_flag) {
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    if (Left_Right_flag == 1) {  // turn left
      // calculate turn radius
      tmp_radius = fabs(final_x_val - vel_pos.x()) /
                   sin(fabs(fabs(theta_final) - fabs(vel_pos.theta())));
      if (tmp_radius < 1 / GetLimitLeftCurv())
        tmp_radius = 1 / GetLimitLeftCurv();
      tmp_radius = fmin(15.0, tmp_radius);
      delta_x = tmp_radius * cos(vel_pos.theta()) + vel_pos.x();
      warningflag = 0;
      flag = 1;
    } else if (Left_Right_flag == 2) {  // turn right
      // calculate turn radius
      tmp_radius = fabs(final_x_val - vel_pos.x()) /
                   sin(fabs(fabs(theta_final) - fabs(vel_pos.theta())));
      if (tmp_radius < 1 / GetLimitRightCurv())
        tmp_radius = 1 / GetLimitRightCurv();
      tmp_radius = fmin(15.0, tmp_radius);
      delta_x = tmp_radius * cos(vel_pos.theta()) + vel_pos.x();
      warningflag = 0;
      flag = -1;
    }
    origin_x = -flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
    origin_y = flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
    // turn
    bool b_carpot_flag = false;
    saved_data_num = 0;
    double tmp_x = vel_pos.x(), tmp_y = vel_pos.y(),
           tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      // check adjust y or x
      if (fabs(tmp_theta) < M_PI_4) {  // move x
        tmp_x += PARK_PRECISION;
        tmp_y =
            -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw(tmp_theta);
      } else {  // move y
        if (Left_Right_flag == 2) {
          tmp_y -= PARK_PRECISION;
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        } else {
          tmp_y += PARK_PRECISION;
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        }
      }

      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vel_pos.set_theta(tmp_theta);
      vel_pos.set_kappa(flag / tmp_radius);      // curvature
      vel_pos.set_direction(0);                  // forward
      vel_pos.set_segment_index(start_segment);  // segment index
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      finish_flag = Satisfystep2Check();
      if (finish_flag == true) {
        abnomous_watcher = 1;
        return;
      }
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag == true) {
        warningflag = true;
        break;
      }

      // save data
      forward_log_data.push_back(vel_pos);
      saved_data_num += 1;
      // check whether to continue
      if (fabs(fabs(theta_final) - fabs(vel_pos.theta())) <= 0.05) {
        b_carpot_flag = true;
        step_2_flag = true;
      }
    }
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    start_segment++;
    i_abnormal_old_watcher = forward_log_data.size();
    if (step_2_flag == true) break;
    // according to warning flag, go straight back, and turn again
    if (saved_data_num < 2) {
      abnomous_watcher = 1;
      return;
    }
    if (warningflag == true) {
      vel_pos = forward_log_data[forward_log_data.size() - 1];
      b_carpot_flag = false;
      delta_x = 0.0;
      tmp_x = vel_pos.x();
      tmp_y = vel_pos.y();
      tmp_theta = vel_pos.theta();
      while (!b_carpot_flag) {
        // insert straight line
        double temp_k = tan(vel_pos.theta());
        double temp_b = vel_pos.y() - temp_k * vel_pos.x();
        // check adjust y or x
        if (fabs(vel_pos.theta()) < M_PI_4) {  // move x
          tmp_x -= PARK_PRECISION;
          tmp_y = temp_k * tmp_x + temp_b;
        } else {  // move y
          if (Left_Right_flag == 2)
            tmp_y += PARK_PRECISION;
          else
            tmp_y -= PARK_PRECISION;
          if (fabs(temp_k) >= 0.01)
            tmp_x = tmp_x;
          else
            tmp_x = (tmp_y - temp_b) / temp_k;
        }
        delta_x += PARK_PRECISION;
        vel_pos.set_x(tmp_x);
        vel_pos.set_y(tmp_y);
        vel_pos.set_theta(tmp_theta);
        vel_pos.set_kappa(0);                      // curvature
        vel_pos.set_direction(1);                  // backward
        vel_pos.set_segment_index(start_segment);  // segment index
        vehicle_pos_.point = vel_pos;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        b_carpot_flag = CarportCurrBoundSafety();
        if (b_carpot_flag == true) break;
        // save data
        forward_log_data.push_back(vel_pos);
        if (delta_x >= 0.5) b_carpot_flag = true;
      }
      vel_pos = forward_log_data[forward_log_data.size() - 1];
      start_segment++;
    }
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}

bool AP_ParkingOut::Satisfystep2Check() {
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
  if (Left_Right_flag == 1) {  // vehicle up,carport down
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
bool AP_ParkingOut::AngularParkingNormalTailOut(
    TrajectoryPoint end_pos, std::vector<TrajectoryPoint>& candidate_pos) {
  // define logging data
  pp_planning_trajectory.clear();
  int start_segment = 0;
  std::vector<TrajectoryPoint> forward_log_data;
  TrajectoryPoint vel_pos;
  vel_pos.Clear();
  vel_pos.set_segment_index(start_segment);
  double tmp_x = vel_pos.x();
  tmp_x = VehicleParam::Instance()->left_front_y() + 0.2 +
          fmax(limit_carport_coor_[2].x(), limit_carport_coor_[3].x());
  vel_pos.set_x(tmp_x);
  vel_pos.set_theta(-1 * M_PI);
  forward_log_data.push_back(vel_pos);
  // define abnomous_watcher
  int abnomous_watcher = 0;

  /*step1: start from carport origin, go backward*/
  NormalTailOutStep1(vel_pos, forward_log_data, start_segment,
                     abnomous_watcher);
  if (forward_log_data.size() < 1) return false;
  /*step2: turn parallel to y-axis*/
  NormalTailOutStep2(vel_pos, forward_log_data, start_segment, abnomous_watcher,
                     end_pos.x(), end_pos.theta());
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
void AP_ParkingOut::NormalTailOutStep1(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher) {
  /*step1.1: start from carport origin, check dis to oppsite dis*/
  double oppsite_dis = 0.0, corner_dis = 0.0;
  ;
  oppsite_dis = fabs(limit_carport_coor_[0].y() - vel_pos.y()) - 1.2;
  corner_dis = fabs(limit_carport_coor_[1].y() - vel_pos.y()) - 1.2;
  if (corner_dis <= 1.6)
    if (oppsite_dis > 0.7) {  // shift
      double shift_target = 0.0;
      if (Left_Right_flag == 1) {
        shift_target = limit_carport_coor_[0].y() + 1.2 + 0.6;
        ShiftYDirectionTailOut(vel_pos, left_radius, right_radius, 1, 1,
                               shift_target, 6.0, forward_log_data,
                               start_segment);
      } else if (Left_Right_flag == 2) {
        shift_target = limit_carport_coor_[0].y() - 1.2 - 0.6;
        ShiftYDirectionTailOut(vel_pos, left_radius, right_radius, 2, 1,
                               shift_target, 6.0, forward_log_data,
                               start_segment);
      }
    }
  double tmp_x = 0.0;
  if (forward_log_data.size() > 1)
    vel_pos = forward_log_data[forward_log_data.size() - 1];
  else {
    vel_pos.Clear();
    tmp_x = VehicleParam::Instance()->left_front_y() + 0.2 +
            fmax(limit_carport_coor_[2].x(), limit_carport_coor_[3].x());
    vel_pos.set_x(tmp_x);
    vel_pos.set_theta(-1 * M_PI);
  }

  /*step1.2: start from carport origin, go backward*/
  double target_x = 0.0;
  target_x = fmax(limit_carport_coor_[0].x(), limit_carport_coor_[1].x()) + 1.0;
  // calculate side area
  vel_pos.set_kappa(0);                      // curvature
  vel_pos.set_segment_index(start_segment);  // segment index
  vel_pos.set_direction(1);                  // backward
  if (vel_pos.x() < target_x - PARK_PRECISION) {
    tmp_x = vel_pos.x();
    while (1) {
      tmp_x += PARK_PRECISION;
      if (tmp_x > target_x) break;
      // save data
      vel_pos.set_x(tmp_x);
      forward_log_data.push_back(vel_pos);
    }
  } else if (vel_pos.x() > target_x + PARK_PRECISION) {
    tmp_x = vel_pos.x();
    while (1) {
      tmp_x -= PARK_PRECISION;
      if (tmp_x < target_x) break;
      // save data
      vel_pos.set_x(tmp_x);
      forward_log_data.push_back(vel_pos);
    }
  }
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  start_segment++;
  return;
}
// turn&out
void AP_ParkingOut::NormalTailOutStep2(
    TrajectoryPoint& vel_pos, std::vector<TrajectoryPoint>& forward_log_data,
    int& start_segment, int& abnomous_watcher, double final_x_val,
    double theta_final) {
  /*step2:turn-forward to a certain angle*/
  RegulateYawHpiHpi(theta_final);
  bool bboardlimit = false;
  abnomous_watcher = 0;
  vel_pos = forward_log_data[forward_log_data.size() - 1];
  vehicle_pos_.point = vel_pos;
  vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
  bboardlimit = CarportCurrBoundSafety();
  if (bboardlimit == true) {
    abnomous_watcher = 1;
    return;
  }
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  int flag = 0;
  double tmp_radius = 0.0;
  bool step_2_flag = true;
  int i_abnormal_watcher = 0;
  int i_abnormal_old_watcher = 0;
  while (step_2_flag) {
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    // 1. right-forward
    if (Left_Right_flag == 2) {  // right-backward
      flag = 1;
      tmp_radius = right_radius;
    } else if (Left_Right_flag == 1) {  // left-backward
      flag = -1;
      tmp_radius = left_radius;
    }
    origin_x = flag * tmp_radius * sin(vel_pos.theta()) + vel_pos.x();
    origin_y = -flag * tmp_radius * cos(vel_pos.theta()) + vel_pos.y();
    b_carpot_flag = false;
    // save data defiine
    vel_pos.set_kappa(-flag / tmp_radius);     // curvature
    vel_pos.set_direction(1);                  // backward
    vel_pos.set_segment_index(start_segment);  // segment index
    double tmp_x = vel_pos.x(), tmp_y = vel_pos.y(),
           tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {
      // first half of segment, move y
      if (fabs(vel_pos.y()) > tmp_radius * 0.5) {
        if (Left_Right_flag == 1) {
          tmp_y -= PARK_PRECISION;
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawHpiHpi(tmp_theta);
        } else if (Left_Right_flag == 2) {
          tmp_y += PARK_PRECISION;
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawHpiHpi(tmp_theta);
        }
      } else {  // second half of segmemt,move x
        tmp_x += PARK_PRECISION;
        tmp_y =
            -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYawHpiHpi(tmp_theta);
      }

      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vel_pos.set_theta(tmp_theta);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        forward_log_data.pop_back();  // give up the previous point
        break;
      }
      // if (SatisfyTailOutStep2Check()) {//anoymous
      //	abnomous_watcher = 1;
      //	return;
      //}
      if (fabs(fabs(theta_final) - fabs(vel_pos.theta())) <= 0.05) {
        b_carpot_flag = true;
        step_2_flag = false;
      }
      forward_log_data.push_back(vel_pos);
    }
    start_segment++;
    if (forward_log_data.size() < 2) {
      abnomous_watcher = 1;
      return;
    }

    vel_pos = forward_log_data[forward_log_data.size() - 1];
    if (step_2_flag == false) break;
    i_abnormal_old_watcher = forward_log_data.size();
    // not finished?
    // 2. straight-backward
    double temp_k = tan(vel_pos.theta());
    double temp_b = vel_pos.y() - temp_k * vel_pos.x();
    double delta_x = 0.0;
    // test
    // go forward to test
    int test_count = 0;
    b_carpot_flag = false;
    tmp_x = vel_pos.x();
    tmp_y = vel_pos.y();
    tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {    // insert straight line
      tmp_x += PARK_PRECISION;  // test go backward
      tmp_y = temp_k * tmp_x + temp_b;
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) break;
      // save data
      test_count++;
      delta_x += PARK_PRECISION;
      if (delta_x >= 1.1) break;
    }

    // execute
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    b_carpot_flag = false;
    vel_pos.set_kappa(0);
    if (test_count <= 5)
      vel_pos.set_direction(0);
    else
      vel_pos.set_direction(1);
    vel_pos.set_segment_index(start_segment);
    delta_x = 0.0;
    tmp_x = vel_pos.x();
    tmp_y = vel_pos.y();
    tmp_theta = vel_pos.theta();
    while (!b_carpot_flag) {  // insert straight line
      if (test_count <= 5)
        tmp_x -= PARK_PRECISION;
      else
        tmp_x += PARK_PRECISION;
      tmp_y = temp_k * tmp_x + temp_b;
      vel_pos.set_x(tmp_x);
      vel_pos.set_y(tmp_y);
      vehicle_pos_.point = vel_pos;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      b_carpot_flag = CarportCurrBoundSafety();
      if (b_carpot_flag) {
        forward_log_data.pop_back();
        break;
      }
      // save data
      forward_log_data.push_back(vel_pos);
      delta_x += PARK_PRECISION;
      if (delta_x >= 0.7) break;
    }
    start_segment++;
    vel_pos = forward_log_data[forward_log_data.size() - 1];
    if (step_2_flag == false) break;
    i_abnormal_watcher = forward_log_data.size();
    if (i_abnormal_watcher - i_abnormal_old_watcher <= 1) {
      abnomous_watcher = 1;
      return;
    }
  }
  return;
}
}  // namespace planning
}  // namespace neodrive
