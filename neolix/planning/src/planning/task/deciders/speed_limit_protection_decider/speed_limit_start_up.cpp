#include "speed_limit_start_up.h"

#include "src/planning/util/speed_limit_trans.h"

namespace neodrive {
namespace planning {

SpeedLimitStartUp::SpeedLimitStartUp()
    : SpeedLimitInterface("speed_limit_start_up") {}

void SpeedLimitStartUp::ComputeSpeedLimit(TaskInfo &task_info) {
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  if (plan_config.speed_limit.enable_start_up_limit &&
      std::fabs(inside_data.vel_v) < 0.5) {
    SaveSpeedLimit(SpeedLimitType::START_UP, SpeedLimitType::SOFT,
                   plan_config.speed_limit.start_up_limit, 0.0);
  }
}

}  // namespace planning
}  // namespace neodrive
