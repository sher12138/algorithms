#include "aeb.h"

#include "config/aeb_config.h"

namespace neodrive {
namespace aeb {
namespace {
double SpeedErrorTolerance(double current_x, double min_x = 10.0 / 3.6,
                           double min_y = 0.0, double max_x = 25.0 / 3.6,
                           double max_y = 7.2 / 3.6) {
  if (current_x <= min_x) return min_y;
  if (current_x >= max_x) return max_y;
  return (current_x - min_x) / (max_x - min_x) * (max_y - min_y);
}
}  // namespace

void Aeb::Init() {
  if (initialized_) {
    return;
  }
  aeb_config_ = AebConfig::Instance();
  common_config_ = neodrive::common::config::CommonConfig::Instance();
  initialized_ = true;
}

void Aeb::RunOnce() {
  CheckInputTimeout();
  FreespaceTranspose();
  CheckForbidAeb();
  AebContext::Instance()->is_aeb_active = JudgeAebActive();
  if (pre_aeb_active_ != AebContext::Instance()->is_aeb_active) {
    LOG_INFO("JudgeAebActive: {}", AebContext::Instance()->is_aeb_active);
    pre_aeb_active_ = AebContext::Instance()->is_aeb_active;
  }
}

void Aeb::CheckForbidAeb() {
  auto &chassis_msg = AebContext::Instance()->chassis_msg;
  auto &forbid_aeb_msg = AebContext::Instance()->forbid_aeb_msg;
  auto &pnc_forbid_msg = AebContext::Instance()->pnc_forbid_aeb_msg;
  bool openapi_forbid{false}, canbus_forbid{false}, pnc_forbid{false},
      config_forbid{false};
  static bool pre_canbus_forbid{false};
  canbus_forbid =
      chassis_msg.is_available && chassis_msg.ptr->remote_forbid_aeb();
  if (pre_canbus_forbid && !canbus_forbid) forbid_aeb_msg.ptr->set_value(false);
  pre_canbus_forbid = canbus_forbid;
  openapi_forbid = forbid_aeb_msg.is_available && forbid_aeb_msg.ptr->value();
  pnc_forbid = pnc_forbid_msg.is_available && pnc_forbid_msg.ptr->value();
  config_forbid = !aeb_config_->aeb_config().aeb_switch;
  AebContext::Instance()->is_forbid_aeb =
      canbus_forbid || openapi_forbid || pnc_forbid || config_forbid;
  if (AebContext::Instance()->is_forbid_aeb) {
    LOG_INFO(
        "canbus_forbid:{},openapi_forbid:{},pnc_forbid:{},config_forbid:{}",
        canbus_forbid, openapi_forbid, pnc_forbid, config_forbid);
  }
}

bool Aeb::CheckIsSkipCondition() {
  auto &chassis_msg = AebContext::Instance()->chassis_msg;
  if (is_chassis_msg_timeout_ ||
      (chassis_msg.ptr->current_vcu_mode() != VCU_ADU &&
       chassis_msg.ptr->current_vcu_mode() != VCU_PDU)) {
    LOG_WARN("not ad and sd skip aeb");
    return true;
  }
  if (!is_chassis_msg_timeout_ &&
      (chassis_msg.ptr->gear_location() == GEAR_PARKING ||
       chassis_msg.ptr->gear_location() == GEAR_REVERSE)) {
    LOG_WARN("Gear:{},skip aeb",
             GearPosition_Name(chassis_msg.ptr->gear_location()));
    return true;
  }
  if (AebContext::Instance()->is_failure) {
    LOG_WARN("Failure,skip aeb");
    return true;
  }
  return false;
}

bool Aeb::JudgeAebActive() {
  auto &chassis_msg = AebContext::Instance()->chassis_msg;
  auto &odom_msg = AebContext::Instance()->odom_msg;
  auto &forbid_aeb_msg = AebContext::Instance()->forbid_aeb_msg;
  current_speed_ =
      (chassis_msg.ptr != nullptr) ? chassis_msg.ptr->speed_mps() : 0.0;
  consider_obstacles_.clear();
  back_obstacles_.clear();
  skip_sectors_.clear();
  auto &imu_in_odom = odom_msg.ptr->odometry_pose();
  current_heading_ = common::GetYawFromQuaternion(imu_in_odom.orientation());
  SelectFrontObstacles();
  if (CheckIsSkipCondition()) {
    return false;
  }
  AebContext::Instance()->is_ego_car_stop =
      (std::fabs(current_speed_) <= kAebStopSpeed) ? true : false;
  LOG_DEBUG("is_ego_car_stop:{},speed:{}",
            AebContext::Instance()->is_ego_car_stop, current_speed_);
  CalculateTurnRadius();
  CalculateStopDist();
  bool obstacle_triggered = false;
  AebObstaclePtr current_obstacle_ptr;
  for (auto &each_obstacle : consider_obstacles_) {
    if (true == CheckObstacle(each_obstacle)) {
      current_obstacle_ptr = each_obstacle;
      LOG_DEBUG("trigger obstacle local position pos_x:{},pos_y:{},vel:{}",
                each_obstacle->local_pos.x(), each_obstacle->local_pos.y(),
                each_obstacle->linear_velocity);
      for (auto &each_polygon_pt : each_obstacle->local_polygon) {
        LOG_DEBUG(each_polygon_pt.DebugString());
      }
      if (AebContext::Instance()->is_ego_car_stop &&
          each_obstacle->linear_velocity <
              aeb_config_->aeb_config().pass_velocity_threshold) {
        LOG_INFO("obstacle triggerd,ego_car_stoped:{}",
                 each_obstacle->linear_velocity);
        obstacle_triggered = true;
        break;
      }
      if (each_obstacle->linear_velocity - current_speed_ +
              SpeedErrorTolerance(current_speed_) >=
          aeb_config_->aeb_config().pass_velocity_threshold) {
        LOG_DEBUG("obstacle filterd by speed:{}",
                  each_obstacle->linear_velocity);
        skip_sectors_.push_back(GetObstacleSector(each_obstacle));
      } else {
        LOG_DEBUG("obstacle triggerd,obstacle slow:{}",
                  each_obstacle->linear_velocity);
        obstacle_triggered = true;
        break;
      }
    } else {
      LOG_DEBUG("obstacle other:{}", each_obstacle->linear_velocity);
      if (each_obstacle->linear_velocity - current_speed_ >=
              aeb_config_->aeb_config().pass_velocity_threshold &&
          each_obstacle->local_pos.y() <=
              aeb_config_->aeb_config()
                  .close_moving_obstacle_filter_threshold) {
        skip_sectors_.push_back(GetObstacleSector(each_obstacle));
      }
    }
  }
  if (obstacle_triggered && current_obstacle_ptr->active_count >= 5) {
    obstacle_trigger_count_ = current_obstacle_ptr->active_count - 1;
  }
  if (AebContext::Instance()->perception_msg.is_updated) {
    obstacle_trigger_count_ =
        obstacle_triggered ? obstacle_trigger_count_ + 1 : 0;
  }
  for (auto &each_obstacle : back_obstacles_) {
    if (each_obstacle->linear_velocity - current_speed_ +
                SpeedErrorTolerance(current_speed_) >=
            aeb_config_->aeb_config().pass_velocity_threshold &&
        each_obstacle->local_pos.y() <=
            aeb_config_->aeb_config().close_moving_obstacle_filter_threshold) {
      skip_sectors_.push_back(GetObstacleSector(each_obstacle));
    }
  }
  if (AebContext::Instance()->freespace_msg.is_available) {
    freespace_trigger_count_ =
        CheckFreespace() ? freespace_trigger_count_ + 1 : 0;
    LOG_INFO("freespace trigger count:{}", freespace_trigger_count_);
  }
  if (obstacle_triggered && freespace_trigger_count_ >= 2)
    LOG_INFO("double check passed");
  if (freespace_trigger_count_ >= 4) LOG_INFO("freespace check passed");
  if (obstacle_trigger_count_ >= 4) LOG_INFO("obstacle check passed");
  return (obstacle_triggered && freespace_trigger_count_ >= 2) ||
         freespace_trigger_count_ >= 4 || obstacle_trigger_count_ >= 4;
}

SectorType Aeb::GetObstacleSector(AebObstaclePtr aeb_obstacle) {
  double min_ang = 2.0 * M_PI, max_ang = -2.0 * M_PI;
  for (auto &each_polygon_pt : aeb_obstacle->local_polygon) {
    double ang = std::atan2(each_polygon_pt.x(), each_polygon_pt.y());
    ang = common::normalize_angle(ang);
    min_ang = std::min(min_ang, ang);
    max_ang = std::max(max_ang, ang);
  }
  LOG_DEBUG("sector:{},{}", min_ang, max_ang);
  return std::make_pair(min_ang, max_ang);
}

void Aeb::FreespaceTranspose() {
  auto &freespace = AebContext::Instance()->freespace_msg;
  auto &freespace_pts = AebContext::Instance()->freespace_pts;
  if (freespace.is_updated) {
    freespace_pts.clear();
    freespace_pts.reserve(freespace.ptr->freespace_size());
    for (int i = 0; i < freespace.ptr->freespace_size(); ++i) {
      auto &origin_pt = freespace.ptr->freespace(i);
      Point2D freespace_pt;
      freespace_pt.set_x(origin_pt.y());
      freespace_pt.set_y(-origin_pt.x());
      freespace_pts.emplace_back(std::move(freespace_pt));
    }
    freespace.is_updated = false;
  }
}

bool Aeb::CheckFreespace() {
  if (is_freespace_msg_timeout_) return false;
  AebContext::Instance()->freespace_msg.is_available = false;
  auto &ego_car_config = common_config_->ego_car_config();
  auto &dynamic_aeb_config = aeb_config_->aeb_dynamic_config().default_group;
  double in_longitudinal_threshold =
      ego_car_config.length - ego_car_config.imu_in_car_x;
  double out_longitudinal_threshold =
      ego_car_config.length - ego_car_config.imu_in_car_x +
      dynamic_aeb_config.longitudianl_expand_dist +
      ((AebContext::Instance()->is_ego_car_stop)
           ? dynamic_aeb_config.stop_safe_dist
           : stop_distance_);
  auto check_point = [&](Point2D &check_pt) -> bool {
    double dist_to_curve_center = common::Distance2D(check_pt, curve_center_);
    dist_to_curve_center +=
        ((curve_center_.y() < 0.0 && check_pt.y() < curve_center_.y()) ||
         (curve_center_.y() > 0.0 && check_pt.y() > curve_center_.y()))
            ? std::fabs(curvature_radius_)
            : 0.0;
    double arc_to_car =
        std::asin(check_pt.x() / dist_to_curve_center) * dist_to_curve_center;
    if (current_speed_ >= 6.944 && arc_to_car >= 3.0) return false;
    return ((arc_to_car >= in_longitudinal_threshold) &&
            (arc_to_car <= out_longitudinal_threshold) &&
            (dist_to_curve_center <= outer_radius_) &&
            (dist_to_curve_center >= inner_radius_));
  };
  auto &freespace_pts = AebContext::Instance()->freespace_pts;
  for (int i = 0; i + 2 < freespace_pts.size(); ++i) {
    auto &current_pt = freespace_pts[i];
    auto &next_pt = freespace_pts[i + 1];
    if (current_pt.x() < 0 && next_pt.x() < 0) continue;
    if (check_point(current_pt) && !CheckIfInSkipSector(current_pt))
      return true;
    if (common::Distance2D(current_pt, next_pt) >
        dynamic_aeb_config.polygon_distance_threshold) {
      std::vector<Point2D> completed_points;
      GeneratePolygonPoints(current_pt, next_pt, completed_points);
      for (auto &each_pt : completed_points) {
        if (check_point(each_pt) && !CheckIfInSkipSector(each_pt)) return true;
      }
    }
  }
  return false;
}

bool Aeb::CheckIfInSkipSector(Point2D &freespace_pt) {
  auto &config = aeb_config_->aeb_config();
  double ang =
      common::normalize_angle(std::atan2(freespace_pt.x(), freespace_pt.y()));
  for (auto &each_sector : skip_sectors_) {
    if (ang >= each_sector.first - config.sector_expand &&
        ang <= each_sector.second + config.sector_expand)
      return true;
  }
  LOG_DEBUG("skip failed:{}", ang);
  return false;
}

void Aeb::CalculateTurnRadius() {
  auto chassis_msg = AebContext::Instance()->chassis_msg.ptr;
  auto &ego_car_config = common_config_->ego_car_config();
  auto &dynamic_aeb_config = aeb_config_->aeb_dynamic_config().default_group;
  double steer = 0.0;
  if (chassis_msg == nullptr) {
    curvature_radius_ = kMaxCurvatureRadius;
  } else if (std::isnan(chassis_msg->steering_percentage())) {
    LOG_ERROR("Missing chassis steering percentage info!");
    curvature_radius_ = kMaxCurvatureRadius;
  } else if (std::fabs(chassis_msg->steering_percentage()) <= 1e-3) {
    curvature_radius_ = kMaxCurvatureRadius;
  } else {
    steer = static_cast<double>(chassis_msg->steering_percentage());
    curvature_radius_ =
        ego_car_config.wheel_base /
        std::tan(steer / 100.0 * ego_car_config.max_steer_angle /
                 ego_car_config.steer_ratio);
    LOG_DEBUG(
        "Vehicle curvature radius is: {}, steer_percent: {}, max_steer_angle "
        "{}, steer_ratio {}, wheel_base {}",
        curvature_radius_, steer, ego_car_config.max_steer_angle,
        ego_car_config.steer_ratio, ego_car_config.wheel_base);
  }
  //(curvature radius < 0.0) ? turn_right : turn_left;
  curve_center_.set_x(ego_car_config.base_link_in_car_x -
                      ego_car_config.imu_in_car_x);
  curve_center_.set_y(curvature_radius_);
  inner_radius_ = std::sqrt(
      std::pow((std::fabs(curvature_radius_) - ego_car_config.width / 4.0 -
                dynamic_aeb_config.lateral_expand_dist),
               2) +
      std::pow(ego_car_config.front_edge_to_base_link, 2));
  outer_radius_ = std::sqrt(
      std::pow((std::fabs(curvature_radius_) + ego_car_config.width / 4.0 +
                dynamic_aeb_config.lateral_expand_dist),
               2) +
      std::pow(ego_car_config.front_edge_to_base_link, 2));
  LOG_DEBUG("inner:{},outer:{}", inner_radius_, outer_radius_);
}

void Aeb::SelectFrontObstacles() {
  // LOG_INFO("============SelectFrontObstacles===============");
  if (AebContext::Instance()->perception_msg.ptr == nullptr) return;
  auto &perception_obstacles = *AebContext::Instance()->perception_msg.ptr;
  auto &imu_in_odom = perception_obstacles.odom_pose();
  LOG_DEBUG("vehicle pose x:{},y:{},heading:{}", imu_in_odom.position().x(),
            imu_in_odom.position().y(), current_heading_);
  for (int i = 0; i < perception_obstacles.odometry_perception_obstacle_size();
       ++i) {
    auto &odom_obstacle = perception_obstacles.odometry_perception_obstacle(i);
    if (odom_obstacle.sub_type() == PerceptionObstacle::ST_TREE) continue;
    AebObstaclePtr each_local_obstacle = std::make_shared<AebObstacle>();
    GetLocalObstacle(imu_in_odom, current_heading_, odom_obstacle,
                     each_local_obstacle);
    if (each_local_obstacle->local_pos.x() > 0) {
      consider_obstacles_.emplace_back(each_local_obstacle);
    } else {
      back_obstacles_.emplace_back(each_local_obstacle);
    }
  }
}

void Aeb::GeneratePolygonPoints(Point2D &start, Point2D &end,
                                std::vector<Point2D> &out_pts) {
  auto &dynamic_aeb_config = aeb_config_->aeb_dynamic_config().default_group;
  auto &ego_car_config = common_config_->ego_car_config();
  double a = start.y() - end.y(), b = end.x() - start.x(),
         c = start.x() * end.y() - end.x() * start.y();
  for (double y = std::min(start.y(), end.y()) +
                  dynamic_aeb_config.polygon_distance_step;
       y < std::max(start.y(), end.y());
       y += dynamic_aeb_config.polygon_distance_step) {
    Point2D new_pt;
    new_pt.set_y(y);
    new_pt.set_x(-b / a * y - c / a);
    if (new_pt.x() > ego_car_config.front_edge_to_base_link) {
      out_pts.emplace_back(new_pt);
    }
  }
}

bool Aeb::CheckObstacle(AebObstaclePtr aeb_obstacle) {
  auto &dynamic_aeb_config = aeb_config_->aeb_dynamic_config().default_group;
  auto &ego_car_config = common_config_->ego_car_config();
  double in_longitudinal_threshold =
      ego_car_config.length - ego_car_config.imu_in_car_x;
  double out_longitudinal_threshold =
      ego_car_config.length - ego_car_config.imu_in_car_x +
      dynamic_aeb_config.longitudianl_expand_dist +
      ((AebContext::Instance()->is_ego_car_stop)
           ? dynamic_aeb_config.stop_safe_dist
           : stop_distance_);
  auto check_point = [&](Point2D &check_pt, double &long_dis) -> bool {
    double dist_to_curve_center = common::Distance2D(check_pt, curve_center_);
    dist_to_curve_center +=
        ((curve_center_.y() < 0.0 && check_pt.y() < curve_center_.y()) ||
         (curve_center_.y() > 0.0 && check_pt.y() > curve_center_.y()))
            ? std::fabs(curvature_radius_)
            : 0.0;
    double arc_to_car =
        std::asin(check_pt.x() / dist_to_curve_center) * dist_to_curve_center;
    if ((dist_to_curve_center <= outer_radius_) &&
        (dist_to_curve_center >= inner_radius_)) {
      long_dis = std::min(long_dis, arc_to_car);
    }
    return ((arc_to_car >= in_longitudinal_threshold) &&
            (arc_to_car <= out_longitudinal_threshold) &&
            (dist_to_curve_center <= outer_radius_) &&
            (dist_to_curve_center >= inner_radius_));
  };
  double check_dist = std::numeric_limits<double>::max();
  double speed_relative = current_speed_ - aeb_obstacle->linear_velocity;
  for (size_t i = 0; i + 1 < aeb_obstacle->local_polygon.size(); ++i) {
    auto &current_pt = aeb_obstacle->local_polygon[i];
    auto &next_pt =
        aeb_obstacle->local_polygon
            [(i + 1 == aeb_obstacle->local_polygon.size()) ? i : i + 1];
    if (check_point(current_pt, check_dist)) {
      double ttc = speed_relative <= 0 ? kMaxTTC
                                       : std::fabs(check_dist) / speed_relative;
      LOG_INFO("obstacle:{} step into check area,ttc:{}", aeb_obstacle->id,
               ttc);
      if (ttc <= aeb_config_->aeb_config().obstacle_collision_ttc) return true;
    }
    if (common::Distance2D(current_pt, next_pt) >
        dynamic_aeb_config.polygon_distance_threshold) {
      std::vector<Point2D> completed_points;
      GeneratePolygonPoints(current_pt, next_pt, completed_points);
      for (auto &each_pt : completed_points) {
        if (check_point(each_pt, check_dist)) {
          double ttc = speed_relative <= 0
                           ? kMaxTTC
                           : std::fabs(check_dist) / speed_relative;
          LOG_INFO("obstacle:{} step into check area,ttc:{}", aeb_obstacle->id,
                   ttc);
          if (ttc <= aeb_config_->aeb_config().obstacle_collision_ttc)
            return true;
        }
      }
    }
  }
  return false;
}

void Aeb::CalculateStopDist() {
  double max_deaccel = common_config_->ego_car_config().aeb_deceleration;
  stop_distance_ = -pow(current_speed_ / max_deaccel, 2) * max_deaccel / 2.0;
  LOG_DEBUG("stop dist: {}", stop_distance_);
}

void Aeb::CheckInputTimeout() {
  auto &chassis_msg = AebContext::Instance()->chassis_msg;
  auto &freespace_msg = AebContext::Instance()->freespace_msg;
  auto &perception_msg = AebContext::Instance()->perception_msg;
  auto now_t = common::Now();
  is_chassis_msg_timeout_ =
      (now_t - chassis_msg.latest_recv_time_sec >= kMsgTimeoutThreshold);
  is_freespace_msg_timeout_ =
      (now_t - freespace_msg.latest_recv_time_sec >= kMsgTimeoutThreshold);
  is_perception_obstacle_msg_timeout_ =
      (now_t - perception_msg.latest_recv_time_sec >= kMsgTimeoutThreshold);
  if (is_perception_obstacle_msg_timeout_ && is_freespace_msg_timeout_) {
    AebContext::Instance()->is_failure = true;
  } else {
    AebContext::Instance()->is_failure = false;
  }
  if (is_chassis_msg_timeout_ || is_freespace_msg_timeout_ ||
      is_perception_obstacle_msg_timeout_) {
    LOG_ERROR(
        "is_chassis_msg_timeout_:{},,is_freespace_msg_"
        "timeout_:{},is_perception_obstacle_msg_timeout_:{}",
        is_chassis_msg_timeout_, is_freespace_msg_timeout_,
        is_perception_obstacle_msg_timeout_);
  }
}
}  // namespace aeb
}  // namespace neodrive
