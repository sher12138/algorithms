#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedSmoothSlowDownDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedSmoothSlowDownDecider);

 public:
  virtual ~SpeedSmoothSlowDownDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);

 private:
  void SmoothSlowDownBeforeObstacle(
      const std::vector<SpeedObstacleDecision>& speed_obstacle_decision,
      const std::unordered_set<int>& dp_st_ignore_static_set);
  void SlowDownProcess();

 private:
  const config::AutoPlanConfig::SpeedSmoothSlowDown*
      speed_smooth_slow_down_config_{nullptr};
  double adc_front_edge_to_center_{0.0};
  double adc_current_v_{0.0};
  bool update_limited_speed_{false};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double all_delay_time_{2.0};  // from perception to vehicle's delay time
  double slow_down_final_distance_{std::numeric_limits<double>::infinity()};

 private:
  const double kApproximateEqualZero{0.0001};
};
REGISTER_SCENARIO_TASK(SpeedSmoothSlowDownDecider);
}  // namespace planning
}  // namespace neodrive