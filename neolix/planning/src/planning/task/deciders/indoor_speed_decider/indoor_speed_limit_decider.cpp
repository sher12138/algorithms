#include "indoor_speed_limit_decider.h"

#include "src/planning/task/deciders/speed_limit_protection_decider/speed_limit_path.h"
#include "src/planning/task/deciders/speed_limit_protection_decider/speed_limit_reference_line.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

IndoorSpeedLimitDecider::IndoorSpeedLimitDecider() {
  name_ = "IndoorSpeedLimitDecider";
}

void IndoorSpeedLimitDecider::Reset() {}

IndoorSpeedLimitDecider::~IndoorSpeedLimitDecider() { Reset(); }

void IndoorSpeedLimitDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode IndoorSpeedLimitDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  SpeedLimitPath path_limit;
  SpeedLimitReferenceLine reference_line_limit;
  path_limit.ComputeSpeedLimit(task_info);
  reference_line_limit.ComputeSpeedLimit(task_info);
  const auto& indoor_config =
      config::PlanningConfig::Instance()->plan_config().indoor;
  double v_limit{indoor_config.max_speed};
  double dis_to_end{data_center_->master_info().distance_to_end()};
  if (dis_to_end < 2.) {
    v_limit =
        dis_to_end <= 0. ? 0. : std::max(0.5, v_limit * (dis_to_end / 2.));
    path_limit.SaveSpeedLimit(SpeedLimitType::STATION_STOP,
                              SpeedLimitType::SOFT, v_limit, 0.0);
  }

  if (std::abs(vehicle_state_.SteerPercent()) >=
      indoor_config.steer_threshold) {
    v_limit = 1.;
    path_limit.SaveSpeedLimit(SpeedLimitType::PATH, SpeedLimitType::SOFT,
                              v_limit, 0.0);
    LOG_INFO("steer percent is {:.2f}", vehicle_state_.SteerPercent());
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
