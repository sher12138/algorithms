#include "motorway_speed_limit_protection_decider.h"

#include "motorway_speed_limit_path.h"
#include "motorway_speed_limit_reference_line.h"
#include "motorway_speed_limit_reverse_lane_detour.h"
#include "motorway_speed_limit_slope.h"
#include "motorway_speed_limit_start_up.h"
#include "motorway_speed_limit_turn_right.h"

namespace neodrive {
namespace planning {

MotorwaySpeedLimitProtectionDecider::MotorwaySpeedLimitProtectionDecider() {
  name_ = "MotorwaySpeedLimitProtectionDecider";
}

MotorwaySpeedLimitProtectionDecider::~MotorwaySpeedLimitProtectionDecider() {
  Reset();
}

ErrorCode MotorwaySpeedLimitProtectionDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>> start execute: {}", name_);

  auto &frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (frame->outside_planner_data().speed_slow_down) {
    LOG_INFO("skip task, speed_slow_down");
    return ErrorCode::PLANNING_OK;
  }

  MotorwaySpeedLimitPath path_limit;
  MotorwaySpeedLimitReferenceLine reference_line_limit;
  MotorwaySpeedLimitSlope slop_limit;
  MotorwaySpeedLimitStartUp start_up_limit;
  MotorwaySpeedLimitReverseLaneDetour reverse_lane_detour_limit;
  MotorwaySpeedLimitTurnRight turn_right_speed_limit;

  path_limit.ComputeSpeedLimit(task_info);
  reference_line_limit.ComputeSpeedLimit(task_info);
  slop_limit.ComputeSpeedLimit(task_info);
  start_up_limit.ComputeSpeedLimit(task_info);
  reverse_lane_detour_limit.ComputeSpeedLimit(task_info);
  turn_right_speed_limit.ComputeSpeedLimit(task_info);

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
