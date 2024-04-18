#include "motorway_speed_perception_obstacle_caution_decider.h"

#include "reference_line/reference_line_util.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

MotorwaySpeedPerceptionObstacleCautionDecider::MotorwaySpeedPerceptionObstacleCautionDecider() {
  name_ = "MotorwaySpeedPerceptionObstacleCautionDecider";
}

MotorwaySpeedPerceptionObstacleCautionDecider::
    ~MotorwaySpeedPerceptionObstacleCautionDecider() {
  Reset();
}

ErrorCode MotorwaySpeedPerceptionObstacleCautionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_SKIP;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  return ErrorCode::PLANNING_OK;
}

void MotorwaySpeedPerceptionObstacleCautionDecider::SaveTaskResults(
    TaskInfo& task_info) {
  if (update_limited_speed_) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::PERCEPTION_CAUTION);
    internal_speed_limit.add_upper_bounds(limited_speed_);
    internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
    internal_speed_limit.set_acceleration(0.0);
    LOG_INFO(
        "PERCEPTION_CAUTION {} limit speed: speed = {:.2f}, acc = {:.2f}",
        SpeedLimit_ConstraintType_Name(internal_speed_limit.constraint_type()),
        limited_speed_, 0.0);

    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
  }
}

void MotorwaySpeedPerceptionObstacleCautionDecider::Reset(){};

bool MotorwaySpeedPerceptionObstacleCautionDecider::Init(TaskInfo& task_info) {
  speed_perception_obstacle_caution_config_ =
      &config::PlanningConfig::Instance()
           ->planning_research_config()
           .speed_perception_obstacle_caution_decider_config;
  if (!speed_perception_obstacle_caution_config_) {
    return false;
  }

  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  if (!update_limited_speed_) {
    last_limited_speed_ = adc_current_v_;
  }
  update_limited_speed_ = false;
  limited_speed_ = std::numeric_limits<double>::infinity();

  path_adc_boxes_.clear();
  collide_s_vec_.clear();

  if (!BuildPathAdcBoundingBoxes(
          task_info.current_frame()->inside_planner_data(),
          *(task_info.current_frame()->outside_planner_data().path_data))) {
    LOG_ERROR("BuildPathAdcBoundingBoxes failed.");
    return false;
  }

  return true;
}

bool MotorwaySpeedPerceptionObstacleCautionDecider::BuildPathAdcBoundingBoxes(
    const InsidePlannerData& inside_data, const PathData& path_data) {
  if (path_data.path().num_of_points() < 2) {
    LOG_ERROR("path_data.path().num_of_points({}) < 2.",
              path_data.path().num_of_points());
    return false;
  }

  const auto& veh_path = path_data.path().path_points();
  double en_large_buffer{
      speed_perception_obstacle_caution_config_->adc_enlarge_buffer};
  double front_en_large_buffer{en_large_buffer};
  double back_en_large_buffer{en_large_buffer};
  if (inside_data.curr_scenario_state == ScenarioState::BACK_OUT) {
    en_large_buffer = front_en_large_buffer = back_en_large_buffer = 0;
  }
  LOG_INFO("current_level, front_buffer, back_buffer: {}, {:.3f}, {:.3f}",
           inside_data.curr_multi_level, front_en_large_buffer,
           back_en_large_buffer);

  for (const auto& path_point : veh_path) {
    const auto adc_box = VehicleParam::Instance()->get_adc_bounding_box(
        {path_point.x(), path_point.y()}, path_point.theta(), en_large_buffer,
        front_en_large_buffer, back_en_large_buffer);
    if (!Utility::check_area(adc_box)) {
      LOG_ERROR("build adc bounding box from path point at s[{}] failed",
                path_point.s());
      return false;
    }
    path_adc_boxes_.emplace_back(adc_box);
  }
  if (path_adc_boxes_.size() != veh_path.size()) {
    LOG_ERROR("path_adc_boxes_.size({}) != veh_path.size({})",
              path_adc_boxes_.size(), veh_path.size());
    return false;
  }

  return true;
}

bool MotorwaySpeedPerceptionObstacleCautionDecider::Process(TaskInfo& task_info) {
  StaticObsCollisionCheck(
      task_info.current_frame()->inside_planner_data(),
      DataCenter::Instance()->caution_speed_odometry_obstacles(),
      task_info.current_frame()->mutable_outside_planner_data());

  if (!collide_s_vec_.empty()) ComputeSpeedLimit(task_info);

  return true;
}

ErrorCode MotorwaySpeedPerceptionObstacleCautionDecider::StaticObsCollisionCheck(
    const InsidePlannerData& inside_data,
    const std::unordered_map<int, Obstacle>& caution_obstacles,
    OutsidePlannerData* const outside_data) {
  for (const auto& [id, obs] : caution_obstacles) {
    CollisionCheckObstacleWithoutTrajectory(inside_data, obs, outside_data);
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode
MotorwaySpeedPerceptionObstacleCautionDecider::CollisionCheckObstacleWithoutTrajectory(
    const InsidePlannerData& inside_data, const Obstacle& obstacle,
    OutsidePlannerData* const outside_data) {
  if (outside_data == nullptr) {
    LOG_ERROR("Input is nullptr.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (obstacle.length() < 1e-4 || obstacle.width() < 1e-4) {
    LOG_WARN("obs {}, length {:.3f}, width {:.3f}", obstacle.id(),
             obstacle.length(), obstacle.width());
    return ErrorCode::PLANNING_OK;
  }
  const auto& path_points = outside_data->path_data->path().path_points();
  if (path_adc_boxes_.size() < 3) {
    LOG_ERROR("adc bounding boxes size < 3.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (path_points.size() != path_adc_boxes_.size()) {
    LOG_ERROR("path_points size != path_adc_boxes_ size.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  double collide_s = path_points.back().s() + 100.0;
  for (std::size_t index = 0; index < path_adc_boxes_.size(); ++index) {
    if (obstacle.polygon().has_overlap(Polygon2d(path_adc_boxes_[index]))) {
      collide_s = path_points[index].s();
      if (index > 0) {
        if (obstacle.polygon().has_overlap(
                Polygon2d(path_adc_boxes_[index - 1]))) {
          collide_s = path_points[index - 1].s();
        }
      }
      LOG_INFO("caution obs {}, collide s {:.3f}", obstacle.id(), collide_s);
      break;
    }
  }
  if (collide_s < path_points.back().s()) {
    collide_s_vec_.emplace_back(collide_s);
  }

  return ErrorCode::PLANNING_OK;
}

bool MotorwaySpeedPerceptionObstacleCautionDecider::ComputeSpeedLimit(
    TaskInfo& task_info) {
  if (collide_s_vec_.empty()) return true;

  double distance_to_stop =
      *std::min_element(collide_s_vec_.begin(), collide_s_vec_.end());
  LOG_INFO("distance_to_stop {:.3f}", distance_to_stop);

  double deceleration =
      (std::pow(adc_current_v_, 2) -
       std::pow(
           speed_perception_obstacle_caution_config_->slow_down_target_speed,
           2)) /
      2.0 / distance_to_stop;
  deceleration = std::min(
      deceleration,
      static_cast<double>(
          speed_perception_obstacle_caution_config_->limit_max_deceleration));
  limited_speed_ = std::min(
      limited_speed_,
      last_limited_speed_ -
          deceleration *
              speed_perception_obstacle_caution_config_->all_delay_time);
  limited_speed_ = std::max(
      limited_speed_,
      static_cast<double>(
          speed_perception_obstacle_caution_config_->slow_down_target_speed));

  update_limited_speed_ = true;
  last_limited_speed_ = limited_speed_;

  return true;
}

}  // namespace planning
}  // namespace neodrive
