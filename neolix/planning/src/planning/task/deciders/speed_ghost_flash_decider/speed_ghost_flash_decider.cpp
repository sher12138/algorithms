#include "speed_ghost_flash_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/math/curve1d/spline.h"
#include "src/planning/util/speed_planner_common.h"

using JunctionType = autobot::cyberverse::Junction::JunctionType;

namespace neodrive {
namespace planning {

namespace {
struct SingleObsInfo {
  double last_lat_dis{std::numeric_limits<double>::infinity()};
  std::size_t closer_cnt{0};
  bool closer_flag{false};
  bool updated{false};
  void Reset() {
    last_lat_dis = std::numeric_limits<double>::infinity();
    closer_cnt = 0;
    closer_flag = false;
    updated = false;
  }
};

struct GhostFlashObsInfo {
  std::unordered_map<int, SingleObsInfo> obs_info{};
  void Reset() { obs_info.clear(); }
};

const config::AutoPlanningResearchConfig::SpeedGhostFlashDeciderConfig*
    speed_ghost_flash_config_{nullptr};
double adc_front_edge_to_center_{0.0};
double adc_current_s_{0.0};
double adc_front_edge_s_{0.0};
double adc_current_l_{0.0};
double adc_current_v_{0.0};
double adc_vel_heading_{0.0};
bool update_limited_speed_{false};
double last_limited_speed_{std::numeric_limits<double>::infinity()};
double limited_speed_{std::numeric_limits<double>::infinity()};
Boundary ghost_flash_check_area_{};
const double kApproximateEqualZero{1e-3};
const double kDoubleMax{1000.0};
GhostFlashObsInfo ghost_flash_obs_info_{};
std::vector<Vec2d> all_types_cross_roads_{};
double limit_min_speed_{1000.0};

// Find all crossroad(+,T,Y) overlapped with (adc_current_s_,
// +check_area_length) on reference-line(except in_roads and Unknown).
void InitAllCrossRoad(const ReferenceLinePtr& reference_line,
                      const double& check_area_length) {
  // reset data
  all_types_cross_roads_.clear();

  // init data
  const auto& junction_list = reference_line->junctions();
  for (const auto& [junction_ptr, overlap] : junction_list) {
    if (!junction_ptr) {
      continue;
    }
    if (overlap.end_s < adc_current_s_) {
      continue;
    }
    if (overlap.start_s > adc_front_edge_s_ + check_area_length) {
      break;
    }
    if ((junction_ptr->Type() ==
         static_cast<uint32_t>(JunctionType::UNKNOWN)) ||
        (junction_ptr->Type() ==
         static_cast<uint32_t>(JunctionType::IN_ROAD))) {
      continue;
    } else {
      all_types_cross_roads_.emplace_back(
          Vec2d{overlap.start_s, overlap.end_s});
    }
  }
  LOG_INFO("all_types_cross_roads_ num {} .", all_types_cross_roads_.size());
}

void UpdateGhostFlashObsInfo(const int id, const double lat_dis) {
  ghost_flash_obs_info_.obs_info[id].updated = true;
  if (ghost_flash_obs_info_.obs_info[id].last_lat_dis - lat_dis >
      kApproximateEqualZero) {
    ghost_flash_obs_info_.obs_info[id].closer_cnt++;
  } else {
    ghost_flash_obs_info_.obs_info[id].closer_cnt = 0;
  }

  if (ghost_flash_obs_info_.obs_info[id].closer_cnt >
      speed_ghost_flash_config_->closer_cnt_threshold) {
    ghost_flash_obs_info_.obs_info[id].closer_cnt =
        speed_ghost_flash_config_->closer_cnt_threshold;
    ghost_flash_obs_info_.obs_info[id].closer_flag = true;
  } else {
    ghost_flash_obs_info_.obs_info[id].closer_flag = false;
  }

  ghost_flash_obs_info_.obs_info[id].last_lat_dis = lat_dis;
}

bool ObsNotOverlapCrossRoad(const Obstacle* const obs) {
  if (all_types_cross_roads_.empty()) {
    return true;
  }

  for (const auto& s_range : all_types_cross_roads_) {
    if (s_range.y() < obs->min_s()) {
      continue;
    } else if (s_range.x() > obs->max_s()) {
      return true;
    } else {
      LOG_INFO("obs [{}] overlap with cross roads.", obs->id());
      return false;
    }
  }

  return true;
}

// Consider pedestrain and bicycle all the time; but operation of
// unknown_movable and vehicle is different according to if it is in crossroad.
bool FilterStaticObs(const Obstacle* const obs) {
  if (!obs) {
    return true;
  }
  if (!ghost_flash_check_area_.has_overlap(obs->PolygonBoundary())) {
    return true;
  }
  if (!(obs->type() == Obstacle::ObstacleType::UNKNOWN_MOVABLE ||
        obs->type() == Obstacle::ObstacleType::PEDESTRIAN ||
        obs->type() == Obstacle::ObstacleType::BICYCLE ||
        obs->type() == Obstacle::ObstacleType::VEHICLE)) {
    return true;
  }
  if ((obs->type() == Obstacle::ObstacleType::UNKNOWN_MOVABLE ||
       obs->type() == Obstacle::ObstacleType::VEHICLE) &&
      speed_ghost_flash_config_->only_consider_cross_road &&
      ObsNotOverlapCrossRoad(obs)) {
    return true;
  }
  return false;
}

bool FilterDynamicObs(const std::unordered_set<int>& has_collision_dynamic_obs,
                      const Obstacle* const obs) {
  if (!obs) {
    return true;
  }
  if (obs->speed() > speed_ghost_flash_config_->dynamic_obs_speed_threshold) {
    return true;
  }
  if (!ghost_flash_check_area_.has_overlap(obs->PolygonBoundary())) {
    return true;
  }

  auto iter = has_collision_dynamic_obs.find(obs->id());
  if (iter != has_collision_dynamic_obs.end()) {
    return true;
  }

  if (!(obs->type() == Obstacle::ObstacleType::UNKNOWN_MOVABLE ||
        obs->type() == Obstacle::ObstacleType::PEDESTRIAN ||
        obs->type() == Obstacle::ObstacleType::BICYCLE ||
        obs->type() == Obstacle::ObstacleType::VEHICLE)) {
    return true;
  }
  return false;
}

double GetSpeedByLatDis(const bool is_obs_closer, const double lat_dis) {
  std::vector<double> lat_dis_list(
      speed_ghost_flash_config_->lat_dis_list.begin(),
      speed_ghost_flash_config_->lat_dis_list.end());
  std::vector<double> speed_limit_list(
      speed_ghost_flash_config_->speed_limit_list.begin(),
      speed_ghost_flash_config_->speed_limit_list.end());

  tk::spline lat_limit_speed_spline{};
  lat_limit_speed_spline.set_points(lat_dis_list, speed_limit_list,
                                    tk::spline::spline_type::linear);
  double speed_at_emergency_point = lat_limit_speed_spline(lat_dis);

  if (is_obs_closer) {
    speed_at_emergency_point /= 2;
  }
  LOG_INFO("speed_at_emergency_point by lat_dis: {:.2f}",
           speed_at_emergency_point);
  return speed_at_emergency_point;
}

double GetSpeedByAdcObsSpeed(const InsidePlannerData& inside_data,
                             const Obstacle* const obs, const double lat_dis) {
  double heading_diff =
      normalize_angle(obs->velocity_heading() - inside_data.vel_heading);
  double obs_speed_parallel_car = obs->speed() * std::cos(heading_diff);
  double obs_speed_vertical_car = obs->speed() * std::sin(heading_diff);
  if (obs->speed() < kApproximateEqualZero ||
      obs_speed_vertical_car < kApproximateEqualZero) {
    return std::numeric_limits<double>::infinity();
  }

  double speed{std::numeric_limits<double>::infinity()};
  if (lat_dis < speed_ghost_flash_config_->lat_attention_dis &&
      obs_speed_vertical_car > kApproximateEqualZero) {
    LOG_INFO("lat_dis is little and obs is moving, limit speed to 0.0.");
    speed = 0.0;
  } else {
    // "collision time of lateral * adc_speed" >
    //"obs->max_s + safe_buffer - adc_current_s"
    speed = lat_dis / obs_speed_vertical_car *
                        (inside_data.vel_v - obs_speed_parallel_car) >
                    obs->max_s() + speed_ghost_flash_config_->obs_buffer -
                        adc_current_s_
                ? std::numeric_limits<double>::infinity()
                : 0.0;
    LOG_INFO("use lat_dis, relative speed , limit speed to {:.3f}.", speed);
  }
  return speed;
}

double GetSpeedAtEmergencyPoint(const InsidePlannerData& inside_data,
                                const Obstacle* const obs, const double lat_dis,
                                double& speed_at_emergency_point) {
  if (speed_ghost_flash_config_->lat_dis_list.size() !=
      speed_ghost_flash_config_->speed_limit_list.size()) {
    return false;
  }

  if (lat_dis < speed_ghost_flash_config_->lat_dis_list.front()) {
    speed_at_emergency_point = 0.0;
    return true;
  } else if (lat_dis > kDoubleMax) {
    speed_at_emergency_point = kDoubleMax;
    return true;
  }

  UpdateGhostFlashObsInfo(obs->id(), lat_dis);

  bool is_obs_closer =
      speed_planner_common::IsApproachingAdc(obs, inside_data) ||
      ghost_flash_obs_info_.obs_info[obs->id()].closer_flag;

  speed_at_emergency_point = GetSpeedByLatDis(is_obs_closer, lat_dis);
  if (is_obs_closer) {
    speed_at_emergency_point =
        std::min(speed_at_emergency_point,
                 GetSpeedByAdcObsSpeed(inside_data, obs, lat_dis));
  }

  return true;
}

double CalDecelerationByObs(const double lon_dis, const double target_speed) {
  double deceleration = 0.0;
  if (lon_dis < kApproximateEqualZero && target_speed < adc_current_v_) {
    deceleration = speed_ghost_flash_config_->limit_max_deceleration;
  } else if (target_speed <= 0.0) {
    deceleration = std::pow(adc_current_v_ - target_speed, 2) / 2.0 / lon_dis;
  } else if (adc_current_v_ > target_speed) {
    deceleration = (std::pow(adc_current_v_, 2) - std::pow(target_speed, 2)) /
                   2.0 / lon_dis;
  }
  LOG_INFO("target_speed, lon_dis, deceleration:{:.2f}, {:.2f}, {:.2f}.",
           target_speed, lon_dis, deceleration);

  return std::max(deceleration, 0.0);
}

void GetDecelerationByStaticObs(TaskInfo& task_info, double& deceleration) {
  const auto& static_obstacle = task_info.current_frame()
                                    ->planning_data()
                                    .decision_data()
                                    .static_obstacle();
  if (static_obstacle.empty()) {
    return;
  }

  for (const auto& obs : static_obstacle) {
    // 1.1 filter obs(localization and type)
    if (FilterStaticObs(obs)) {
      continue;
    }

    // 1.2 get target speed of emergency point, according to lat_dis。
    const auto& inside_data = task_info.current_frame()->inside_planner_data();
    double obs_adc_lat_dis =
        speed_planner_common::GetObs2AdcLateralDis(*obs, adc_current_l_);
    double obs_path_lat_dis{0.0};
    double lat_dis = speed_planner_common::GetObsToPathLatDis(
                         task_info.current_frame()
                             ->outside_planner_data()
                             .speed_obstacle_context.adc_sl_boundaries,
                         *obs, obs_path_lat_dis)
                         ? obs_path_lat_dis
                         : obs_adc_lat_dis;
    LOG_INFO(
        "obs id type:[{}], [{}]; obs_adc_lat_dis, obs_path_lat_dis, lat_dis: "
        "{:.3f}, {:.3f}, {:.3f} ",
        obs->id(), static_cast<int>(obs->type()), obs_adc_lat_dis,
        obs_path_lat_dis, lat_dis);
    double speed_at_emergency_point{0.0};
    if (!GetSpeedAtEmergencyPoint(inside_data, obs, lat_dis,
                                  speed_at_emergency_point)) {
      continue;
    }

    // 1.3 get target deceleration, according to target speed and lon_dis.
    double lon_dis = obs->min_s() - adc_front_edge_s_ -
                     speed_ghost_flash_config_->obs_buffer;
    // 2as=v0^2-v^2
    deceleration = std::max(
        deceleration, CalDecelerationByObs(lon_dis, speed_at_emergency_point));
  }
}

void GetDecelerationByDynamicObs(TaskInfo& task_info, double& deceleration) {
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  if (dynamic_obstacle.empty()) {
    return;
  }

  std::unordered_set<int> has_collision_dynamic_obs{};
  const auto& dynamic_obs_collision_info =
      task_info.current_frame()
          ->outside_planner_data()
          .speed_obstacle_context.dynamic_obstacles_decision;
  for (const auto& info : dynamic_obs_collision_info) {
    if (info.lower_points.size() && info.upper_points.size()) {
      has_collision_dynamic_obs.insert(info.obstacle.id());
    }
  }

  for (const auto& obs : dynamic_obstacle) {
    // 1.1 filter obs(localization and type)
    if (FilterDynamicObs(has_collision_dynamic_obs, obs)) {
      continue;
    }

    // 1.2 get target speed of emergency point, according to lat_dis。
    const auto& inside_data = task_info.current_frame()->inside_planner_data();
    double obs_adc_lat_dis =
        speed_planner_common::GetObs2AdcLateralDis(*obs, adc_current_l_);
    double obs_path_lat_dis{0.0};
    double lat_dis = speed_planner_common::GetObsToPathLatDis(
                         task_info.current_frame()
                             ->outside_planner_data()
                             .speed_obstacle_context.adc_sl_boundaries,
                         *obs, obs_path_lat_dis)
                         ? obs_path_lat_dis
                         : obs_adc_lat_dis;
    LOG_INFO(
        "obs [{}] obs_adc_lat_dis, obs_path_lat_dis, lat_dis: {:.3f}, {:.3f}, "
        "{:.3f} ",
        obs->id(), obs_adc_lat_dis, obs_path_lat_dis, lat_dis);

    double speed_at_emergency_point{0.0};
    if ((obs->type() == Obstacle::ObstacleType::PEDESTRIAN ||
         obs->type() == Obstacle::ObstacleType::BICYCLE) &&
        lat_dis < speed_ghost_flash_config_->dynamic_obs_safe_dis) {
      speed_at_emergency_point = limit_min_speed_;
      LOG_INFO("low speed pedestrain or bicycle, limit speed to {:.3f}.",
               limit_min_speed_);
    } else if (!GetSpeedAtEmergencyPoint(inside_data, obs, lat_dis,
                                         speed_at_emergency_point)) {
      continue;
    }

    // 1.3 get target deceleration, according to target speed and lon_dis.
    LOG_INFO("consider dynamic obs [{}].", obs->id());
    double lon_dis = obs->min_s() - adc_front_edge_s_ -
                     speed_ghost_flash_config_->obs_buffer;
    // 2as=v0^2-v^2
    deceleration = std::max(
        deceleration, CalDecelerationByObs(lon_dis, speed_at_emergency_point));
  }
}

}  // namespace

SpeedGhostFlashDecider::SpeedGhostFlashDecider() {
  name_ = "SpeedGhostFlashDecider";
}

SpeedGhostFlashDecider::~SpeedGhostFlashDecider() { Reset(); }

ErrorCode SpeedGhostFlashDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void SpeedGhostFlashDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::GHOST_FLASH);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "GHOST_FLASH {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);

    last_limited_speed_ = limited_speed_;
  }
}

void SpeedGhostFlashDecider::Reset() {}

bool SpeedGhostFlashDecider::Init(TaskInfo& task_info) {
  speed_ghost_flash_config_ = &config::PlanningConfig::Instance()
                                   ->planning_research_config()
                                   .speed_ghost_flash_decider_config;
  if (!speed_ghost_flash_config_) {
    return false;
  }
  limit_min_speed_ = speed_ghost_flash_config_->limit_min_speed_ratio *
                     DataCenter::Instance()->drive_strategy_max_speed();
  adc_vel_heading_ =
      task_info.current_frame()->inside_planner_data().vel_heading;
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  adc_front_edge_to_center_ = VehicleParam::Instance()->front_edge_to_center();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ = adc_current_s_ + adc_front_edge_to_center_;
  adc_current_l_ = task_info.curr_sl().l();
  double check_area_length = std::max(
      adc_current_v_ * speed_ghost_flash_config_->check_area_time,
      static_cast<double>(speed_ghost_flash_config_->check_area_length));
  ghost_flash_check_area_ = Boundary(
      adc_current_s_, adc_current_s_ + check_area_length,
      adc_current_l_ - (speed_ghost_flash_config_->check_area_width / 2.0),
      adc_current_l_ + (speed_ghost_flash_config_->check_area_width / 2.0));

  decltype(ghost_flash_obs_info_.obs_info) tmp{};
  for (auto& [id, info] : ghost_flash_obs_info_.obs_info) {
    if (info.updated) {
      info.updated = false;
      tmp[id] = info;
    }
  }
  std::swap(ghost_flash_obs_info_.obs_info, tmp);

  InitAllCrossRoad(task_info.reference_line(), check_area_length);

  return true;
}

bool SpeedGhostFlashDecider::Process(TaskInfo& task_info) {
  // 1. Calculata max deceleration by lat_dis between obs and adc, obs and path.
  //(Only consider obs overlapped with ghost_flash_check_area_.)

  double deceleration{0.0};
  GetDecelerationByStaticObs(task_info, deceleration);
  GetDecelerationByDynamicObs(task_info, deceleration);

  // 2. get limited speed according to max deceleration.
  if (deceleration > kApproximateEqualZero) {
    // limit max deceleration,in case length of obs changes fast.
    deceleration = std::min(
        deceleration,
        static_cast<double>(speed_ghost_flash_config_->limit_max_deceleration));
    update_limited_speed_ = true;
    limited_speed_ =
        std::min(limited_speed_,
                 last_limited_speed_ -
                     deceleration * speed_ghost_flash_config_->all_delay_time);
    limited_speed_ =
        std::max(limited_speed_, static_cast<double>(limit_min_speed_));
    LOG_INFO(
        "final val: deceleration, "
        "last_limited_speed_, limited_speed_:{:.2f}, {:.2f}, {:.2f} .",
        deceleration, last_limited_speed_, limited_speed_);
    limited_speed_ = std::max(limited_speed_, 0.0);
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
