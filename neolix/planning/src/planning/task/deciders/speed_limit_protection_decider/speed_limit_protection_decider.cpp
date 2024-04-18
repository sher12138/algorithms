#include "speed_limit_protection_decider.h"

#include "speed_limit_path.h"
#include "speed_limit_reference_line.h"
#include "speed_limit_reverse_lane_detour.h"
#include "speed_limit_slope.h"
#include "speed_limit_start_up.h"

namespace neodrive {
namespace planning {

SpeedLimitProtectionDecider::SpeedLimitProtectionDecider() {
  name_ = "SpeedLimitProtectionDecider";
}

SpeedLimitProtectionDecider::~SpeedLimitProtectionDecider() { Reset(); }

ErrorCode SpeedLimitProtectionDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>> start execute: {}", name_);

  auto &frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().speed_slow_down) {
    LOG_INFO("skip task, speed_slow_down");
    return ErrorCode::PLANNING_OK;
  }

  SpeedLimitPath path_limit;
  SpeedLimitReferenceLine reference_line_limit;
  SpeedLimitSlope slop_limit;
  SpeedLimitStartUp start_up_limit;
  SpeedLimitReverseLaneDetour reverse_lane_detour_limit;

  path_limit.ComputeSpeedLimit(task_info);
  reference_line_limit.ComputeSpeedLimit(task_info);
  slop_limit.ComputeSpeedLimit(task_info);
  start_up_limit.ComputeSpeedLimit(task_info);
  reverse_lane_detour_limit.ComputeSpeedLimit(task_info);

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
