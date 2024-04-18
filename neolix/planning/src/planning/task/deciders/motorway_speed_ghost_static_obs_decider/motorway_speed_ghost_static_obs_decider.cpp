#include "motorway_speed_ghost_static_obs_decider.h"

namespace neodrive {
namespace planning {

MotorwaySpeedGhostStaticObsDecider::MotorwaySpeedGhostStaticObsDecider() {
  name_ = "MotorwaySpeedGhostStaticObsDecider";
}

MotorwaySpeedGhostStaticObsDecider::~MotorwaySpeedGhostStaticObsDecider() {
  Reset();
}

ErrorCode MotorwaySpeedGhostStaticObsDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
