#include "motorway_speed_ghost_bus_harbor_decider.h"

namespace neodrive {
namespace planning {

MotorwaySpeedGhostBusHarborDecider::MotorwaySpeedGhostBusHarborDecider() {
  name_ = "MotorwaySpeedGhostBusHarborDecider";
}

MotorwaySpeedGhostBusHarborDecider::~MotorwaySpeedGhostBusHarborDecider() {
  Reset();
}

ErrorCode MotorwaySpeedGhostBusHarborDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
