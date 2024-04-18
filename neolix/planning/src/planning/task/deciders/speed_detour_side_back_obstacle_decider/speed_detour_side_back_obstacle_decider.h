#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
constexpr double kDistanceBuff = 5.0;
constexpr double kSpeedBuffer = 0.5;

struct DangerousObsInfo {
  int id{};
  Boundary obs_boundary;
  bool reverse{false};
  double s{0.0};
  double t{0.0};
  double ideal_v{0.0};
  bool skip{false};
};
class SpeedDetourSideBackObstacleDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedDetourSideBackObstacleDecider);
  using CruiseStageState = neodrive::global::planning::CruiseStageState;

 public:
  virtual ~SpeedDetourSideBackObstacleDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  bool IsLaneDetourArea(TaskInfo& task_info);

  bool GetConflictAreaInfo(TaskInfo& task_info);

  bool DynamicObsCheck(TaskInfo& task_info,
                       const std::vector<Obstacle*>& dynamic_obs_vec);

  bool STDecision(TaskInfo& task_info);

  bool UpdateSpeedLimit(TaskInfo& task_info);

  bool IsDynamicObsNeedIgnord(TaskInfo& task_info, const Obstacle* obs,
                              bool& is_near_obs);

  bool BorrowRegionCollideCheck(TaskInfo& task_info, const Obstacle* const obs);

  bool IgnoreObsByBackBoundary(TaskInfo& task_info, const Obstacle* obs);

  bool IgnoreObsByLateralDistance(TaskInfo& task_info, const Obstacle* obs);

  bool IgnoreObsBySpeed(const Obstacle* obs, bool& is_near_obs);

  std::size_t PathIndex(TaskInfo& task_info, const double s);

  double GetStopSAtFrenetPath(TaskInfo& task_info, const double l);

 private:
  double conflict_area_start_s_{0.0};
  double conflict_area_stop_s_{0.0};
  double adc_current_v_{0.0};
  double adc_front_edge_s_{0.0};
  double ref_v_{
      std::numeric_limits<double>::infinity()};  // used to set speed limit
  SLPoint init_sl_point_;
  int final_obs_id_{-1};
  Boundary back_attention_boundary_{};
  bool stop_flag_{false};
  std::vector<DangerousObsInfo> dangerous_obs_;
  bool has_collide_risk_{false};
};

REGISTER_SCENARIO_TASK(SpeedDetourSideBackObstacleDecider);

}  // namespace planning
}  // namespace neodrive