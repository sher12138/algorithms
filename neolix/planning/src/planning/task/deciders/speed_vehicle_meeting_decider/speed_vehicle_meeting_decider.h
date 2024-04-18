#pragma once

#include <unordered_set>

#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedVehicleMeetingDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedVehicleMeetingDecider);

 public:
  virtual ~SpeedVehicleMeetingDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  bool JudgeHasCollisionWithEGO(TaskInfo& task_info, const Obstacle* const obs);

 private:
  double closest_vehcile_meeting_distance_{};
  double adc_current_v_{0.0};
  // all lane meetings on reference line after ego car.
  std::unordered_set<LaneMeetingRanges::ConstPtr> lane_meeting_ranges_{};
  // std::unordered_set<uint64_t> lane_id_set_{};
  std::unordered_map<uint64_t, std::vector<ReferencePoint>> lane_id_s_{};
  // following four vectors: same obs, same index.
  std::vector<int> attention_ids_{};
  std::vector<double> attention_project_v_{};
  std::vector<double> attention_start_s_{};
  std::vector<double> thw_info_{};
  std::vector<double> attention_obs_overlap_in_time_{};
  std::vector<double> attention_obs_overlap_out_time_{};
  std::vector<double> ego_overlap_start_s_in_time_{};
  std::vector<double> ego_overlap_start_s_out_time_{};
  double speed_limit_{};
  double accelerate_{};
  bool update_speed_limit_{false};

  double adc_current_s_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};

  // int run_time_in_cycle_{-1};

 private:
  const double kApproximateEqualZero{1e-4};
};
REGISTER_SCENARIO_TASK(SpeedVehicleMeetingDecider);
}  // namespace planning
}  // namespace neodrive
