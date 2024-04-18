#include "speed_smooth_slow_down_decider.h"

#include "common/visualizer_event/visualizer_event.h"
#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {
SpeedSmoothSlowDownDecider::SpeedSmoothSlowDownDecider() {
  name_ = "SpeedSmoothSlowDownDecider";
}

SpeedSmoothSlowDownDecider::~SpeedSmoothSlowDownDecider() { Reset(); }

ErrorCode SpeedSmoothSlowDownDecider::Execute(TaskInfo& task_info) {
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

void SpeedSmoothSlowDownDecider::SaveTaskResults(TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::SMOOTH_PARKING);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "SMOOTH_PARKING {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void SpeedSmoothSlowDownDecider::Reset(){};

bool SpeedSmoothSlowDownDecider::Init(TaskInfo& task_info) {
  speed_smooth_slow_down_config_ =
      &config::PlanningConfig::Instance()->plan_config().speed_smooth_slow_down;
  if (!speed_smooth_slow_down_config_) {
    return false;
  }
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();
  slow_down_final_distance_ = std::numeric_limits<double>::infinity();
  adc_front_edge_to_center_ = VehicleParam::Instance()->front_edge_to_center();
  all_delay_time_ = speed_smooth_slow_down_config_->all_delay_time;
  return true;
}

bool SpeedSmoothSlowDownDecider::Process(TaskInfo& task_info) {
  const auto& outside_data = task_info.current_frame()->outside_planner_data();
  const auto& static_obstacles_decision =
      outside_data.speed_obstacle_context.static_obstacles_decision;
  const auto& virtual_obstacle_decision =
      outside_data.speed_obstacle_context.virtual_obstacle_decision;
  const auto& dp_st_ignore_static_set =
      outside_data.speed_obstacle_context.dp_st_map_ignore_static_obs_id;

  SmoothSlowDownBeforeObstacle(static_obstacles_decision,
                               dp_st_ignore_static_set);
  SmoothSlowDownBeforeObstacle(virtual_obstacle_decision,
                               dp_st_ignore_static_set);
  SlowDownProcess();
  return true;
}

void SpeedSmoothSlowDownDecider::SmoothSlowDownBeforeObstacle(
    const std::vector<SpeedObstacleDecision>& speed_obstacle_decision,
    const std::unordered_set<int>& dp_st_ignore_static_set) {
  double slow_down_min_distance{std::numeric_limits<double>::infinity()};
  for (const auto& obstacle_decision : speed_obstacle_decision) {
    const auto& obstacle = obstacle_decision.obstacle;
    if (obstacle.length() < 1.0e-4 || obstacle.width() < 1.0e-4) {
      LOG_ERROR("Obstacle [{}] length({:.4f}) < 1e-4 || width({:.4f}) < 1e-4",
                obstacle.id(), obstacle.length(), obstacle.width());
      continue;
    }
    if (dp_st_ignore_static_set.count(obstacle.id())) {
      continue;
    }

    double curr_slow_down_distance =
        obstacle.is_virtual()
            ? obstacle_decision.lower_points.front().first.s() -
                  speed_smooth_slow_down_config_->virtual_obs_slow_down_buffer
            : obstacle_decision.lower_points.front().first.s() -
                  speed_smooth_slow_down_config_->static_obs_slow_down_buffer;
    LOG_INFO("obs, virtual, curr_slow_down_distance:  [{}], {}, {:.4f}",
             obstacle.id(), obstacle.is_virtual(), curr_slow_down_distance);
    slow_down_min_distance =
        std::min(slow_down_min_distance, curr_slow_down_distance);
  }
  slow_down_final_distance_ =
      std::min(slow_down_final_distance_, slow_down_min_distance);
}

void SpeedSmoothSlowDownDecider::SlowDownProcess() {
  if ((adc_current_v_ <=
       speed_smooth_slow_down_config_->slow_down_target_speed) ||
      std::isinf(slow_down_final_distance_)) {
    LOG_INFO("adc_current_v_,slow_down_final_distance_:{:.2f}, {:.2f}",
             adc_current_v_, slow_down_final_distance_);
    return;
  }

  LOG_INFO("slow_down_final_distance_:{:.2f}", slow_down_final_distance_);
  if (adc_front_edge_to_center_ >= slow_down_final_distance_) {
    limited_speed_ = speed_smooth_slow_down_config_->slow_down_target_speed;
    update_limited_speed_ = true;
    return;
  }

  double distance_to_slow_down =
      slow_down_final_distance_ - adc_front_edge_to_center_;
  double deceleration =
      (std::pow(adc_current_v_, 2) -
       std::pow(speed_smooth_slow_down_config_->slow_down_target_speed, 2)) /
      2.0 / distance_to_slow_down;
  deceleration =
      std::min(deceleration,
               static_cast<double>(
                   speed_smooth_slow_down_config_->limit_max_deceleration));
  limited_speed_ = std::min(
      limited_speed_, last_limited_speed_ - deceleration * all_delay_time_);
  limited_speed_ =
      std::max(limited_speed_,
               static_cast<double>(
                   speed_smooth_slow_down_config_->slow_down_target_speed));
  update_limited_speed_ = true;
  last_limited_speed_ = limited_speed_;
  LOG_INFO(
      "adc slow down, adc_current_v_, slow_down_target_speed, "
      "distance_to_slow_down, limited_speed_: {:.2f}, {:.2f}, {:.2f}, {:.2f} .",
      adc_current_v_, speed_smooth_slow_down_config_->slow_down_target_speed,
      distance_to_slow_down, limited_speed_);
  return;
}

}  // namespace planning
}  // namespace neodrive
