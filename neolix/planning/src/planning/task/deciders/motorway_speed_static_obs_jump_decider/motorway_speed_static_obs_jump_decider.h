#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

struct StaticObsDisInfo {
  double path_lat_dis{-0.1};
  double adc_cartisian_dis{-0.1};
  double s{-0.1};
};

class MotorwaySpeedStaticObsJumpDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedStaticObsJumpDecider);

 public:
  virtual ~MotorwaySpeedStaticObsJumpDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);

 private:
  bool DataCheck(TaskInfo& task_info);
  void ComputeCollideInfo(TaskInfo& task_info);

  double ComputeSpeedLimit(int obs_id, const StaticObsDisInfo& collision_info);
  double ComputeEndSpeedLimit(int obs_id,
                              const StaticObsDisInfo& collision_info);
  void ClearObsHistoryInfo();
  void UpdataObsHistoryInfo(TaskInfo& task_info);

 private:
  const config::AutoPlanConfig::SpeedStaticObsJump*
      speed_static_obs_jump_config_{nullptr};
  double multi_level_enlarge_buffer_{0.3};
  double max_speed_{0.};
  double adc_current_v_{0.0};
  double check_dis_{1.e6};

  Boundary adc_boundary_{};
  std::unordered_map<int, StaticObsDisInfo> collide_infos_{};
  std::unordered_map<int, int> history_dynamic_obs_{};

  std::array<double, 3> init_state_{};

  double speed_limit_{0.};
  bool update_limited_speed_{false};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedStaticObsJumpDecider);

}  // namespace planning
}  // namespace neodrive
