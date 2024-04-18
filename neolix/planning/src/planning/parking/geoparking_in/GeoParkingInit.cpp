#include "GeoParkingInit.h"

#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {
GeoParkingInit::GeoParkingInit() {}

GeoParkingInit::~GeoParkingInit() {}

double GeoParkingInit::GetLimitLeftCurv() const { return Limit_left_curv; }

double GeoParkingInit::GetLimitRightCurv() const { return Limit_right_curv; }

void GeoParkingInit::ClearParkInit() {
  pp_planning_trajectory.clear();
  pp_planning_key_trajectory.clear();
  pp_old_trajectory.clear();
  pp_old_earth_trajectory.clear();
  limit_carport_coor_.clear();

  // carport and road
  limit_carport_coor_.clear();
  limit_road_oppsite_coor_.clear();
  limit_road_rear_coor_.clear();
  limit_road_front_coor_.clear();
  limit_carport_coor_global_.clear();
  limit_road_oppsite_global_.clear();
  limit_road_rear_global_.clear();
  limit_road_front_global_.clear();
  return;
}

void GeoParkingInit::SetPARK_PRECISION(const double precision) {
  PARK_PRECISION = precision;
}

bool GeoParkingInit::Initialize(
    const TrajectoryPoint &vehicle_pos, const TrajectoryPoint &carspot_pos,
    const std::vector<Vec2d> &limit_carport_coor_global,
    const std::vector<Vec2d> &limit_road_oppsite_global,
    const std::vector<Vec2d> &limit_road_rear_global,
    const std::vector<Vec2d> &limit_road_front_global) {
  normal_left_curv = 0.20;
  normal_right_curv = 0.20;
  left_radius = 1 / normal_left_curv;
  right_radius = 1 / normal_right_curv;

  Limit_left_curv = 0.23;
  Limit_right_curv = 0.23;

  protected_half_width = VehicleParam::Instance()->width() * 0.5 + 0.2;

  ClearParkInit();
  vehicle_pos_.point = vehicle_pos;
  final_carport_pos_ = carspot_pos;
  limit_carport_coor_global_ = limit_carport_coor_global;
  limit_road_oppsite_global_ = limit_road_oppsite_global;
  limit_road_rear_global_ = limit_road_rear_global;
  limit_road_front_global_ = limit_road_front_global;
  return SetCarportCoordinates();
}

bool GeoParkingInit::SetCarportCoordinates() {
  limit_carport_coor_.clear();
  limit_road_oppsite_coor_.clear();
  limit_road_rear_coor_.clear();
  limit_road_front_coor_.clear();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  size_t len = limit_carport_coor_global_.size();
  double end_x = final_carport_pos_.x(), end_y = final_carport_pos_.y();
  double end_theta = final_carport_pos_.theta();
  for (size_t i = 0; i < len; ++i) {
    auto &tmp_point = limit_carport_coor_global_[i];
    earth2vehicle(end_x, end_y, end_theta, tmp_point.x(), tmp_point.y(), 0.0,
                  tmp_x, tmp_y, tmp_theta);
    limit_carport_coor_.push_back({tmp_x, tmp_y});
  }
  len = limit_road_oppsite_global_.size();
  for (size_t i = 0; i < len; ++i) {
    auto &tmp_point = limit_road_oppsite_global_[i];
    earth2vehicle(end_x, end_y, end_theta, tmp_point.x(), tmp_point.y(), 0.0,
                  tmp_x, tmp_y, tmp_theta);
    limit_road_oppsite_coor_.push_back({tmp_x, tmp_y});
  }
  len = limit_road_rear_global_.size();
  for (size_t i = 0; i < len; ++i) {
    auto &tmp_point = limit_road_rear_global_[i];
    earth2vehicle(end_x, end_y, end_theta, tmp_point.x(), tmp_point.y(), 0.0,
                  tmp_x, tmp_y, tmp_theta);
    limit_road_rear_coor_.push_back({tmp_x, tmp_y});
  }
  len = limit_road_front_global_.size();
  for (size_t i = 0; i < len; ++i) {
    auto &tmp_point = limit_road_front_global_[i];
    earth2vehicle(end_x, end_y, end_theta, tmp_point.x(), tmp_point.y(), 0.0,
                  tmp_x, tmp_y, tmp_theta);
    limit_road_front_coor_.push_back({tmp_x, tmp_y});
  }
  return BuildRoadCarportLimit();
}

bool GeoParkingInit::BuildRoadCarportLimit() {
  if (limit_road_oppsite_coor_.size() < 2 || limit_road_rear_coor_.size() < 2 ||
      limit_road_front_coor_.size() < 2 || limit_carport_coor_.size() < 3)
    return false;
  Available_area_.vertex_.clear();
  // add road
  std::vector<Vec2d>::iterator iter = limit_road_oppsite_coor_.begin();
  for (; iter != limit_road_oppsite_coor_.end(); ++iter)
    Available_area_.vertex_.push_back(*iter);
  // add forward carport road
  iter = limit_road_front_coor_.end() - 1;
  for (; iter != limit_road_front_coor_.begin(); --iter)
    Available_area_.vertex_.push_back(*iter);
  // add carport
  iter = limit_carport_coor_.begin();
  ++iter;
  for (; iter != limit_carport_coor_.end(); ++iter)
    Available_area_.vertex_.push_back(*iter);
  iter = limit_carport_coor_.begin();
  Available_area_.vertex_.push_back(*iter);
  // add rear carport road
  iter = limit_road_rear_coor_.end() - 2;
  for (; iter != limit_road_rear_coor_.begin(); --iter)
    Available_area_.vertex_.push_back(*iter);
  iter = limit_road_rear_coor_.begin();
  Available_area_.vertex_.push_back(*iter);
  return true;
}

void GeoParkingInit::RegulateYaw(double &yaw_angle) {  //[-pi/2,pi/2]
  // while (yaw_angle > M_PI / 2 + 0.01) yaw_angle -= M_PI;
  // while (yaw_angle < -M_PI / 2 - 0.01) yaw_angle += M_PI;
  if (yaw_angle > M_PI / 2 + 0.01)
    yaw_angle -= M_PI;
  else if (yaw_angle < -M_PI / 2 - 0.01)
    yaw_angle += M_PI;
  return;
}

void GeoParkingInit::RegulateYawPi0(double &yaw_angle) {  //[-pi,0]
  while (yaw_angle > 0) yaw_angle -= M_PI;
  while (yaw_angle < -M_PI - 0.01) yaw_angle += M_PI;
  return;
}

void GeoParkingInit::RegulateYaw0Pi(double &yaw_angle) {  //[0,pi]
  while (yaw_angle < 0) yaw_angle += M_PI;
  while (yaw_angle > M_PI + 0.01) yaw_angle -= M_PI;
  return;
}

void GeoParkingInit::RegulateYawPiPi(double &yaw_angle) {  //[-pi,pi]
  while (yaw_angle < -M_PI - 0.01) yaw_angle += M_PI;
  while (yaw_angle > M_PI + 0.01) yaw_angle -= M_PI;
  return;
}

void GeoParkingInit::RegulateYawHpiHpi(
    double &yaw_angle) {  //[-1.5pi,-0.5pi],for head in
  while (yaw_angle < -1.5 * M_PI - 0.01) yaw_angle += M_PI;
  while (yaw_angle > -0.5 * M_PI + 0.01) yaw_angle -= M_PI;
  return;
}

// flag:x,y
double GeoParkingInit::GetMaxfromCarport(const char flag,
                                         const std::vector<Vec2d> &carport) {
  double tmp_value = -1000.0;
  size_t inum = carport.size();
  switch (flag) {
    case 'x':
      for (size_t i = 0; i < inum; ++i) {
        if (tmp_value < carport[i].x()) tmp_value = carport[i].x();
      }
      break;
    case 'y':
      for (size_t i = 0; i < inum; ++i) {
        if (tmp_value < carport[i].y()) tmp_value = carport[i].y();
      }
      break;
    default:
      break;
  }
  return tmp_value;
}

// flag:x,y
double GeoParkingInit::GetMinfromCarport(const char flag,
                                         const std::vector<Vec2d> &carport) {
  double tmp_value = 1000.0;
  size_t inum = carport.size();
  switch (flag) {
    case 'x':
      for (size_t i = 0; i < inum; ++i) {
        if (tmp_value > carport[i].x()) tmp_value = carport[i].x();
      }
      break;
    case 'y':
      for (size_t i = 0; i < inum; ++i) {
        if (tmp_value > carport[i].y()) tmp_value = carport[i].y();
      }
      break;
    default:
      break;
  }
  return tmp_value;
}

bool GeoParkingInit::CarportCurrBoundSafety() {
  bool bcollide = false;
  size_t inum = vehicle_pos_.rect.vertex_.size();
  for (size_t i = 0; i < inum; ++i) {
    bcollide = Available_area_.PointInPolygon(vehicle_pos_.rect.vertex_[i].x(),
                                              vehicle_pos_.rect.vertex_[i].y());
    if (bcollide == false) return true;
  }
  inum = Available_area_.vertex_.size();
  for (size_t i = 0; i < inum; ++i) {
    bcollide = vehicle_pos_.rect.PointInPolygon(Available_area_.vertex_[i].x(),
                                                Available_area_.vertex_[i].y());
    if (bcollide == true) return true;
  }
  return false;
}

void GeoParkingInit::SaveDatatoBehavior(
    std::vector<TrajectoryPoint> &candidate_pos) {
  candidate_pos = pp_planning_trajectory;

  // calculate dis
  double tmp_dis = 0.0;
  for (std::size_t i = 1; i < candidate_pos.size(); ++i) {
    tmp_dis = sqrt(pow(candidate_pos[i - 1].x() - candidate_pos[i].x(), 2) +
                   pow(candidate_pos[i - 1].y() - candidate_pos[i].y(), 2)) +
              candidate_pos[i - 1].s();
    candidate_pos[i].set_s(tmp_dis);
    pp_planning_trajectory[i].set_s(tmp_dis);
  }
  // transform&save the trajectory to old earth
  if (candidate_pos.size() > 1) {
    TrajectoryPoint tmp_earth;
    std::vector<TrajectoryPoint>::iterator iter = candidate_pos.begin();
    pp_old_earth_trajectory.clear();
    double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
    double end_x = final_carport_pos_.x(), end_y = final_carport_pos_.y();
    double end_theta = final_carport_pos_.theta();
    for (; iter != candidate_pos.end(); ++iter) {
      tmp_earth = *iter;
      vehicle2earth(end_x, end_y, end_theta, iter->x(), iter->y(),
                    iter->theta(), tmp_x, tmp_y, tmp_theta);
      tmp_earth.set_x(tmp_x);
      tmp_earth.set_y(tmp_y);
      tmp_earth.set_theta(tmp_theta);
      pp_old_earth_trajectory.push_back(tmp_earth);
    }
    candidate_pos = pp_old_earth_trajectory;
  }
  return;
}

/*fabs(yaw)<0.1*/
// head_direction-1: head to end;2:tail to end
void GeoParkingInit::MoveXDirection(
    const TrajectoryPoint &start_vel_pos, const TrajectoryPoint &end_vel_pos,
    std::vector<TrajectoryPoint> &forward_log_data, int &start_segment,
    const int head_direction) {
  if (std::abs(start_vel_pos.x() - end_vel_pos.x()) < PARK_PRECISION) return;
  TrajectoryPoint curr_vel_pos(start_vel_pos);
  double x_start = curr_vel_pos.x(), xx_start = x_start;
  double y_start = curr_vel_pos.y(), theta_start = curr_vel_pos.theta();
  double x_end = end_vel_pos.x();
  double y_end = end_vel_pos.y(), theta_end = end_vel_pos.theta();
  double delta_theta = 0.0, temp_b = 0.0, delta_y = 0.0;
  int inum = static_cast<int>(floor(fabs(x_end - x_start) / PARK_PRECISION));
  if (inum > 1) {
    delta_theta = (theta_end - theta_start) / inum;
    delta_y = (y_end - y_start) / inum;
  }
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();

  tmp_theta = atan2(y_end - y_start, x_end - x_start);
  RegulateYaw(tmp_theta);
  temp_b = y_start - tan(tmp_theta) * x_start;
  curr_vel_pos.set_theta(tmp_theta);
  // saved data
  curr_vel_pos.set_kappa(0);  // curvature
  if (head_direction == 2)
    curr_vel_pos.set_direction(1);  // backward
  else
    curr_vel_pos.set_direction(0);                // forward
  curr_vel_pos.set_segment_index(start_segment);  // segment index

  if (x_start < x_end - PARK_PRECISION) {  // go forward
    tmp_x = curr_vel_pos.x();
    tmp_y = curr_vel_pos.y();
    tmp_theta = curr_vel_pos.theta();
    while (xx_start < x_end) {
      tmp_x += PARK_PRECISION;
      tmp_y = delta_y + forward_log_data[forward_log_data.size() - 1].y();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYaw(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      xx_start = curr_vel_pos.x();
    }
  } else if (x_start > x_end + PARK_PRECISION) {  // go backward
    tmp_x = curr_vel_pos.x();
    tmp_y = curr_vel_pos.y();
    tmp_theta = curr_vel_pos.theta();
    while (xx_start > x_end) {
      tmp_x -= PARK_PRECISION;
      tmp_y = delta_y + forward_log_data[forward_log_data.size() - 1].y();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYaw(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      xx_start = curr_vel_pos.x();
    }
  }
  ++start_segment;
  return;
}

// yaw: -pi to pi; head_direction-1: head to end;2:tail to end
void GeoParkingInit::MoveYDirection(
    const TrajectoryPoint &start_vel_pos, const TrajectoryPoint &end_vel_pos,
    std::vector<TrajectoryPoint> &forward_log_data, int &start_segment,
    const int head_direction) {
  if (std::abs(start_vel_pos.y() - end_vel_pos.y()) < PARK_PRECISION) return;
  TrajectoryPoint curr_vel_pos(start_vel_pos);
  double x_start = curr_vel_pos.x();
  double y_start = curr_vel_pos.y(), yy_start = y_start,
         theta_start = curr_vel_pos.theta();
  double x_end = end_vel_pos.x();
  double y_end = end_vel_pos.y(), theta_end = end_vel_pos.theta();
  double delta_theta = 0.0, temp_b = 0.0, delta_x = 0.0;
  int inum = static_cast<int>(floor(fabs(y_end - y_start) / PARK_PRECISION));
  if (inum > 0) {
    delta_theta = (theta_end - theta_start) / inum;
    delta_x = (x_end - x_start) / inum;
  }
  double tmp_x = end_vel_pos.x(), tmp_y = end_vel_pos.y(),
         tmp_theta = end_vel_pos.theta();
  tmp_theta = atan2(y_end - y_start, x_end - x_start);
  RegulateYawPiPi(tmp_theta);
  temp_b = y_start - tan(tmp_theta) * x_start;
  // saved data
  curr_vel_pos.set_theta(tmp_theta);
  curr_vel_pos.set_kappa(0);  // curvature
  if (head_direction == 2)
    curr_vel_pos.set_direction(1);  // backward
  else
    curr_vel_pos.set_direction(0);                // forward
  curr_vel_pos.set_segment_index(start_segment);  // segment index
  if (y_start < y_end - PARK_PRECISION) {         // go forward
    tmp_x = curr_vel_pos.x();
    tmp_y = curr_vel_pos.y();
    tmp_theta = curr_vel_pos.theta();
    while (yy_start < y_end) {
      tmp_y += PARK_PRECISION;
      tmp_x = delta_x + forward_log_data[forward_log_data.size() - 1].x();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYawPiPi(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      yy_start = curr_vel_pos.y();
    }
  } else if (y_start > y_end + PARK_PRECISION) {  // go backward
    tmp_x = curr_vel_pos.x();
    tmp_y = curr_vel_pos.y();
    tmp_theta = curr_vel_pos.theta();
    while (yy_start > y_end) {
      tmp_y -= PARK_PRECISION;
      tmp_x = delta_x + forward_log_data[forward_log_data.size() - 1].x();
      // recalculate heading
      tmp_theta =
          delta_theta + forward_log_data[forward_log_data.size() - 1].theta();
      RegulateYawPiPi(tmp_theta);
      curr_vel_pos.set_x(tmp_x);
      curr_vel_pos.set_y(tmp_y);
      curr_vel_pos.set_theta(tmp_theta);
      forward_log_data.push_back(curr_vel_pos);
      yy_start = curr_vel_pos.y();
    }
  }
  ++start_segment;
  return;
}

/*fabs(yaw)<0.1
leri_flag:first turn left or right. 1:left;2:right;
carport_direc:1:left;2:right to the carport
*/
// vel_coor:current pos; tmp_radius: turn radius
// log data and segment, abnomous_watcher
void GeoParkingInit::ShiftXDirectionOnRoad(
    TrajectoryPoint &vel_coor, const double left_radius,
    const double right_radius, const int leri_flag, const int carport_direc,
    const double target_x_val,
    std::vector<TrajectoryPoint> &in_forward_log_data, int &in_start_segment,
    int &abnomous_watcher) {
  abnomous_watcher = 0;
  bool step_flag = true;
  int flag = 0;
  double old_xx_offset = 0.0, xx_offset = 0.0;
  double tmp_radius = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int i_abnormal_old_watcher = 0, i_abnormal_watcher = 0;
  int dirction_flag = 1;  // y-axis move up or down; 1:up;2:down
  if (carport_direc == 2) dirction_flag = -1;  // down
  int left_right_shift_flag = 0;  // less/larger to final pos;1:less;2:larger
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  if (vel_coor.x() < target_x_val)
    left_right_shift_flag = 1;
  else
    left_right_shift_flag = 2;
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  // TODO(wyc): caution
  while (step_flag) {
    // 1. right-forward
    if (leri_flag == 1) {  // left->right
      flag = -1;
      tmp_radius = left_radius;
    } else if (leri_flag == 2) {  // right->left
      flag = 1;
      tmp_radius = right_radius;
    }
    vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
    origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
    origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
    old_xx_offset = vel_coor.x();
    xx_offset = (old_xx_offset + target_x_val) * 0.5;
    b_carpot_flag = false;
    // log data define
    vel_coor.set_kappa(-flag / tmp_radius);        // curvature
    vel_coor.set_direction(0);                     // forward
    vel_coor.set_segment_index(in_start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {
      tmp_y += dirction_flag * PARK_PRECISION;
      if (carport_direc == 2) {
        tmp_x =
            flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
            origin_x;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYawPi0(tmp_theta);
      } else if (carport_direc == 1) {
        tmp_x =
            -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
            origin_x;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw0Pi(tmp_theta);
      }
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      if (left_right_shift_flag == 1 && tmp_x >= xx_offset)
        b_carpot_flag = true;
      else if (left_right_shift_flag == 2 && tmp_x <= xx_offset)
        b_carpot_flag = true;
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        in_forward_log_data.pop_back();
        break;
      }
      // save data
      in_forward_log_data.push_back(vel_coor);
    }
    in_start_segment++;
    vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
    i_abnormal_old_watcher = in_forward_log_data.size();

    // 2. left-forward
    if (leri_flag == 1) {  // left->right
      flag = 1;
      tmp_radius = right_radius;
    } else if (leri_flag == 2) {  // right->left
      flag = -1;
      tmp_radius = left_radius;
    }
    origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
    origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
    b_carpot_flag = false;
    // log data define
    vel_coor.set_kappa(-flag / tmp_radius);        // curvature
    vel_coor.set_direction(0);                     // forward
    vel_coor.set_segment_index(in_start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {
      tmp_y += dirction_flag * PARK_PRECISION;
      if (carport_direc == 2) {
        tmp_x =
            flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
            origin_x;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYawPi0(tmp_theta);
      } else if (carport_direc == 1) {
        tmp_x =
            -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
            origin_x;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw0Pi(tmp_theta);
      }
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      if (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.02) b_carpot_flag = true;
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        in_forward_log_data.pop_back();
        break;
      }
      // save data
      in_forward_log_data.push_back(vel_coor);
    }
    in_start_segment++;
    vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
    if (left_right_shift_flag == 1 &&
        ((vel_coor.x() >= target_x_val - PARK_PRECISION) &&
         (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.025))) {
      step_flag = false;
      break;
    } else if (left_right_shift_flag == 2 &&
               ((vel_coor.x() <= target_x_val + PARK_PRECISION) &&
                (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.025))) {
      step_flag = false;
      break;
    }
    // not finished?
    if (fabs(vel_coor.x() - target_x_val) > 0.2) {
      // 3. right-backward
      if (leri_flag == 1) {  // left->right
        flag = -1;
        tmp_radius = left_radius;
      } else if (leri_flag == 2) {  // right->left
        flag = 1;
        tmp_radius = right_radius;
      }
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
      origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      old_xx_offset = vel_coor.x();
      xx_offset = (old_xx_offset + target_x_val) * 0.5;
      b_carpot_flag = false;
      vel_coor.set_kappa(-flag / tmp_radius);        // curvature
      vel_coor.set_direction(1);                     // backward
      vel_coor.set_segment_index(in_start_segment);  // segment index
      tmp_x = vel_coor.x();
      tmp_y = vel_coor.y();
      tmp_theta = vel_coor.theta();
      while (!b_carpot_flag) {
        tmp_y -= dirction_flag * PARK_PRECISION;
        if (carport_direc == 2) {
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        } else if (carport_direc == 1) {
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        }
        vel_coor.set_x(tmp_x);
        vel_coor.set_y(tmp_y);
        vel_coor.set_theta(tmp_theta);
        vehicle_pos_.point = vel_coor;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        if (left_right_shift_flag == 1 && vel_coor.x() >= xx_offset)
          b_carpot_flag = true;
        else if (left_right_shift_flag == 2 && vel_coor.x() <= xx_offset)
          b_carpot_flag = true;
        bboardlimit = CarportCurrBoundSafety();
        if (bboardlimit == true) {
          in_forward_log_data.pop_back();
          break;
        }
        // save data
        in_forward_log_data.push_back(vel_coor);
      }
      in_start_segment++;
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];

      // 4. left-backward
      if (leri_flag == 1) {  // left->right
        flag = 1;
        tmp_radius = right_radius;
      } else if (leri_flag == 2) {  // right->left
        flag = -1;
        tmp_radius = left_radius;
      }
      origin_x = flag * tmp_radius * sin(vel_coor.theta()) + vel_coor.x();
      origin_y = -flag * tmp_radius * cos(vel_coor.theta()) + vel_coor.y();
      b_carpot_flag = false;
      vel_coor.set_kappa(-flag / tmp_radius);        // curvature
      vel_coor.set_direction(1);                     // backward
      vel_coor.set_segment_index(in_start_segment);  // segment index
      tmp_x = vel_coor.x();
      tmp_y = vel_coor.y();
      tmp_theta = vel_coor.theta();

      while (!b_carpot_flag) {
        tmp_y -= dirction_flag * PARK_PRECISION;
        if (carport_direc == 2) {
          tmp_x =
              flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYawPi0(tmp_theta);
        } else if (carport_direc == 1) {
          tmp_x =
              -flag * sqrt(tmp_radius * tmp_radius - pow(tmp_y - origin_y, 2)) +
              origin_x;
          tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
          RegulateYaw0Pi(tmp_theta);
        }
        vel_coor.set_x(tmp_x);
        vel_coor.set_y(tmp_y);
        vel_coor.set_theta(tmp_theta);
        vehicle_pos_.point = vel_coor;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        if (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.02) b_carpot_flag = true;
        bboardlimit = CarportCurrBoundSafety();
        if (bboardlimit == true) {
          in_forward_log_data.pop_back();
          break;
        }
        // save data
        in_forward_log_data.push_back(vel_coor);
      }
      in_start_segment++;
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
      i_abnormal_watcher = in_forward_log_data.size();
      if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
        abnomous_watcher = 1;
        return;
      }
      if (left_right_shift_flag == 1 &&
          ((vel_coor.x() >= target_x_val - PARK_PRECISION) &&
           (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.025))) {
        step_flag = false;
        break;
      } else if (left_right_shift_flag == 2 &&
                 ((vel_coor.x() <= target_x_val + PARK_PRECISION) &&
                  (fabs(fabs(vel_coor.theta()) - M_PI_2) <= 0.025))) {
        step_flag = false;
        break;
      }
    }
  }
  return;
}

// vel_coor:current pos; tmp_radius: turn radius
// left_right_flag:1:left;2:right;forward_back_flag:1:forward;2:backward;
void GeoParkingInit::ShiftYDirectionOnRoad(
    TrajectoryPoint &vel_coor, const double left_radius,
    const double right_radius, const int left_right_flag,
    const int forward_back_flag, const double target_y_val,
    std::vector<TrajectoryPoint> &in_forward_log_data, int &in_start_segment,
    int &abnomous_watcher) {  // log data and segment
  abnomous_watcher = 0;
  bool step_flag = true;
  int flag = 0;
  double tmp_radius = 0.0;
  double old_yy_offset = 0.0, yy_offset = 0.0;
  double origin_x = 0.0, origin_y = 0.0;
  bool b_carpot_flag = false;
  bool bboardlimit = false;
  int i_abnormal_old_watcher = 0, i_abnormal_watcher = 0;
  int dirction_flag = 1;
  if (forward_back_flag == 2) dirction_flag = -1;
  int left_right_shift_flag = 0;  // less/larger to final pos;1:less;2:larger
  vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
  if (vel_coor.y() < target_y_val)
    left_right_shift_flag = 1;
  else
    left_right_shift_flag = 2;
  double tmp_x = vel_coor.x(), tmp_y = vel_coor.y(),
         tmp_theta = vel_coor.theta();
  while (step_flag) {
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
    yy_offset = (old_yy_offset + target_y_val) * 0.5;
    vel_coor.set_kappa(-flag / tmp_radius);  // curvature
    if (forward_back_flag == 2)
      vel_coor.set_direction(1);  // backward
    else
      vel_coor.set_direction(0);                   // forward
    vel_coor.set_segment_index(in_start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();
    while (!b_carpot_flag) {
      tmp_x += dirction_flag * PARK_PRECISION;
      tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      if (left_right_shift_flag == 1 && vel_coor.y() >= yy_offset)
        b_carpot_flag = true;
      else if (left_right_shift_flag == 2 && vel_coor.y() <= yy_offset)
        b_carpot_flag = true;
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        // in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // save data
      in_forward_log_data.push_back(vel_coor);
    }
    in_start_segment++;
    if (in_forward_log_data.size() > 1)
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
    else {
      abnomous_watcher = 1;
      return;
    }
    i_abnormal_old_watcher = in_forward_log_data.size();

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

    vel_coor.set_kappa(-flag / tmp_radius);  // curvature
    if (forward_back_flag == 2)
      vel_coor.set_direction(1);  // backward
    else
      vel_coor.set_direction(0);                   // forward
    vel_coor.set_segment_index(in_start_segment);  // segment index
    tmp_x = vel_coor.x();
    tmp_y = vel_coor.y();
    tmp_theta = vel_coor.theta();

    while (!b_carpot_flag) {
      tmp_x += dirction_flag * PARK_PRECISION;
      tmp_y = flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
              origin_y;
      tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
      RegulateYaw(tmp_theta);
      vel_coor.set_x(tmp_x);
      vel_coor.set_y(tmp_y);
      vel_coor.set_theta(tmp_theta);
      vehicle_pos_.point = vel_coor;
      vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
      if (fabs(vel_coor.theta()) < 0.02) b_carpot_flag = true;
      bboardlimit = CarportCurrBoundSafety();
      if (bboardlimit == true) {
        // in_forward_log_data.pop_back();
        in_forward_log_data.pop_back();
        break;
      }
      // save data
      in_forward_log_data.push_back(vel_coor);
    }
    in_start_segment++;
    vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
    if (left_right_shift_flag == 1 &&
        ((vel_coor.y() >= target_y_val - PARK_PRECISION) &&
         (fabs(vel_coor.theta()) < 0.025))) {
      step_flag = false;
      break;
    } else if (left_right_shift_flag == 2 &&
               ((vel_coor.y() <= target_y_val + PARK_PRECISION) &&
                (fabs(vel_coor.theta()) < 0.025))) {
      step_flag = false;
      break;
    }
    // not finished?
    if (fabs(vel_coor.y() - target_y_val) > 0.2) {
      // 3. right-backward
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
      yy_offset = (old_yy_offset + target_y_val) * 0.5;
      b_carpot_flag = false;

      vel_coor.set_kappa(-flag / tmp_radius);  // curvature
      if (forward_back_flag == 2)
        vel_coor.set_direction(0);  // backward
      else
        vel_coor.set_direction(1);                   // forward
      vel_coor.set_segment_index(in_start_segment);  // segment index
      tmp_x = vel_coor.x();
      tmp_y = vel_coor.y();
      tmp_theta = vel_coor.theta();

      while (!b_carpot_flag) {
        tmp_x -= dirction_flag * PARK_PRECISION;
        tmp_y =
            flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw(tmp_theta);
        vel_coor.set_x(tmp_x);
        vel_coor.set_y(tmp_y);
        vel_coor.set_theta(tmp_theta);
        vehicle_pos_.point = vel_coor;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        if (left_right_shift_flag == 1 && vel_coor.y() >= yy_offset)
          b_carpot_flag = true;
        else if (left_right_shift_flag == 2 && vel_coor.y() <= yy_offset)
          b_carpot_flag = true;
        bboardlimit = CarportCurrBoundSafety();
        if (bboardlimit == true) {
          //	in_forward_log_data.pop_back();
          in_forward_log_data.pop_back();
          break;
        }
        // save data
        in_forward_log_data.push_back(vel_coor);
      }
      in_start_segment++;
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];

      // 4. left-backward
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

      vel_coor.set_kappa(-flag / tmp_radius);  // curvature
      if (forward_back_flag == 2)
        vel_coor.set_direction(0);  // backward
      else
        vel_coor.set_direction(1);                   // forward
      vel_coor.set_segment_index(in_start_segment);  // segment index
      tmp_x = vel_coor.x();
      tmp_y = vel_coor.y();
      tmp_theta = vel_coor.theta();

      while (!b_carpot_flag) {
        tmp_x -= dirction_flag * PARK_PRECISION;
        tmp_y =
            flag * sqrt(tmp_radius * tmp_radius - pow(tmp_x - origin_x, 2)) +
            origin_y;
        tmp_theta = atan2(origin_x - tmp_x, tmp_y - origin_y);
        RegulateYaw(tmp_theta);
        vel_coor.set_x(tmp_x);
        vel_coor.set_y(tmp_y);
        vel_coor.set_theta(tmp_theta);
        vehicle_pos_.point = vel_coor;
        vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
        if (fabs(vel_coor.theta()) < 0.02) b_carpot_flag = true;
        bboardlimit = CarportCurrBoundSafety();
        if (bboardlimit == true) {
          //	in_forward_log_data.pop_back();
          in_forward_log_data.pop_back();
          break;
        }
        // save data
        in_forward_log_data.push_back(vel_coor);
      }
      in_start_segment++;
      vel_coor = in_forward_log_data[in_forward_log_data.size() - 1];
      i_abnormal_watcher = in_forward_log_data.size();
      if (i_abnormal_watcher - i_abnormal_old_watcher <= 0) {
        abnomous_watcher = 1;
        return;
      }
      if (left_right_shift_flag == 1 &&
          ((vel_coor.y() >= target_y_val - PARK_PRECISION) &&
           (fabs(vel_coor.theta()) < 0.025))) {
        step_flag = false;
        break;
      } else if (left_right_shift_flag == 2 &&
                 ((vel_coor.y() <= target_y_val + PARK_PRECISION) &&
                  (fabs(vel_coor.theta()) < 0.025))) {
        step_flag = false;
        break;
      }
    }
  }
  return;
}

bool GeoParkingInit::LogParkingData() {
  std::ofstream fp_area("../test_data/output/parking/Parkingarea.csv",
                        std::ios::out);  // only read
  if (!fp_area.is_open()) {
    return false;
  }
  fp_area.setf(std::ios::fixed, std::ios::floatfield);
  fp_area.precision(2);
  /*fp<<"x,y \n";*/
  size_t count_num = Available_area_.vertex_.size();
  double tmp_x = 0.0, tmp_y = 0.0, tmp_theta = 0.0;
  for (size_t i = 0; i < count_num; ++i) {
    auto &pt = Available_area_.vertex_[i];
    vehicle2earth(final_carport_pos_.x(), final_carport_pos_.y(),
                  final_carport_pos_.theta(), pt.x(), pt.y(), 0.0, tmp_x, tmp_y,
                  tmp_theta);
    fp_area << tmp_x << "," << tmp_y << "\n";
  }
  fp_area.close();
  fp_area.clear();

  std::ofstream fp("../test_data/output/parking/ParkingFile.csv",
                   std::ios::out);  // only read
  if (!fp.is_open()) {
    return false;
  }
  fp.setf(std::ios::fixed, std::ios::floatfield);
  fp.precision(2);
  /*fp<<"x,y,yaw,\
  curvature,distance,direction,\
  segment index,vel,accel, \
  x1,y1,x2,y2,x3,y3,x4,y4"<<'\n';*/
  count_num = pp_old_earth_trajectory.size();
  for (size_t i = 0; i < count_num; ++i) {
    vehicle_pos_.point = pp_old_earth_trajectory[i];
    vehicle_shape_creater_.CreateVehicleRectLocal(vehicle_pos_);
    fp << vehicle_pos_.point.x() << "," << vehicle_pos_.point.y() << ","
       << vehicle_pos_.point.theta() << "," << vehicle_pos_.point.kappa() << ","
       << vehicle_pos_.point.s() << "," << vehicle_pos_.point.direction() << ","
       << vehicle_pos_.point.segment_index() << ","
       << vehicle_pos_.point.velocity() << ","
       << vehicle_pos_.point.acceleration() << ",";
    size_t tmp_num = vehicle_pos_.rect.vertex_.size();
    for (size_t j = 0; j < tmp_num; ++j)
      fp << vehicle_pos_.rect.vertex_[j].x() << ","
         << vehicle_pos_.rect.vertex_[j].y() << ",";
    fp << '\n';
  }
  fp.close();
  fp.clear();
  return true;
}

}  // namespace planning
}  // namespace neodrive
