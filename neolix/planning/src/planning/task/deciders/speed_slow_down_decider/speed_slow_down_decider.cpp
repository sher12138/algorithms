#include "speed_slow_down_decider.h"

#include "src/planning/config/planning_config.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

SpeedSlowDownDecider::SpeedSlowDownDecider() { name_ = "SpeedSlowDownDecider"; }

SpeedSlowDownDecider::~SpeedSlowDownDecider() { Reset(); }

ErrorCode SpeedSlowDownDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  lms_sensor_check_failed_ = false;
  road_bound_safe_check_failed_failed_ = false;
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().speed_slow_down) {
    LOG_INFO("skip task, speed_slow_down");
    return ErrorCode::PLANNING_OK;
  }

  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  SlowDown(task_info);

  return ErrorCode::PLANNING_OK;
}

void SpeedSlowDownDecider::SaveTaskResults(TaskInfo& task_info) {
  if (road_bound_risk_slow_down_) {
    const auto& curr_speed =
        DataCenter::Instance()->vehicle_state_proxy().LinearVelocity();
    auto& plan_config = config::PlanningConfig::Instance()->plan_config();
    if (curb_risk_slow_down_) {
      neodrive::global::planning::SpeedLimit internal_speed_limit{};
      internal_speed_limit.set_source_type(SpeedLimitType::ROAD_BOUND);
      internal_speed_limit.add_upper_bounds(0.0);
      internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
      internal_speed_limit.set_acceleration(0.0);
      LOG_INFO("ROAD_BOUND {} limit speed: speed = {:.2f}, acc = {:.2f}",
               SpeedLimit_ConstraintType_Name(
                   internal_speed_limit.constraint_type()),
               0.0, 0.0);

      data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
          internal_speed_limit);
    } else {
      neodrive::global::planning::SpeedLimit internal_speed_limit{};
      internal_speed_limit.set_source_type(SpeedLimitType::ROAD_BOUND);
      internal_speed_limit.add_upper_bounds(0.5);
      internal_speed_limit.set_constraint_type(SpeedLimitType::SOFT);
      internal_speed_limit.set_acceleration(0.0);
      LOG_INFO("ROAD_BOUND {} limit speed: speed = {:.2f}, acc = {:.2f}",
               SpeedLimit_ConstraintType_Name(
                   internal_speed_limit.constraint_type()),
               0.5, 0.0);

      data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
          internal_speed_limit);
    }

    auto outside_planner_data =
        task_info.current_frame()->mutable_outside_planner_data();
    outside_planner_data->path_slow_down_check_failed = true;
    outside_planner_data->lms_sensor_check_failed = lms_sensor_check_failed_;
    outside_planner_data->road_bound_safe_check_failed =
        road_bound_safe_check_failed_failed_;
    LOG_INFO(
        "path_slow_down_check_failed: {} "
        "lms_sensor_check_failed: {} road_bound_safe_check_failed:{}",
        outside_planner_data->path_slow_down_check_failed,
        outside_planner_data->lms_sensor_check_failed,
        outside_planner_data->road_bound_safe_check_failed);
  }
}

bool SpeedSlowDownDecider::SlowDown(TaskInfo& task_info) {
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  const auto& outside_planner_info =
      task_info.current_frame()->outside_planner_data();
  auto& path_data =
      task_info.current_frame()->mutable_outside_planner_data()->path_data;

  // max decel calculate
  speed_planner_common::MaxDecelCalculator(inside_planner_data,
                                           outside_planner_info);

  // lms slow down
  if (FLAGS_planning_enable_lms) {
    bool fail_safe_flag = false;
    speed_planner_common::LmsSensorCheck(inside_planner_data, path_data,
                                         fail_safe_flag);
    if (fail_safe_flag) {
      if (std::fabs(inside_planner_data.vel_v) <
          FLAGS_planning_adc_stop_velocity_threshold) {
        LOG_INFO("slow down: lms sensor check.");
        lms_sensor_check_failed_ = true;
        return true;
      }
    }
  }

  // road bound checker
  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  if (plan_config.speed_limit.enable_road_bound_limit) {
    road_bound_risk_slow_down_ = false;
    curb_risk_slow_down_ = false;
    if (!speed_planner_common::RoadBoundSafeCheck(
            task_info.reference_line(), inside_planner_data,
            *(task_info.current_frame()->mutable_outside_planner_data()),
            path_data, road_bound_risk_slow_down_)) {
      road_bound_safe_check_failed_failed_ = true;
      LOG_INFO("slow down: road bound safe check failed.");
      return true;
    }
    if (road_bound_risk_slow_down_) {
      LOG_INFO("road bound may be not safe!");
      if (!speed_planner_common::PerceptionCurbSafeCheck(inside_planner_data,
                                                         path_data)) {
        curb_risk_slow_down_ = true;
        LOG_INFO("slow down: perception curb safe check.");
        return true;
      }
    }
  }

  return false;
}

}  // namespace planning
}  // namespace neodrive