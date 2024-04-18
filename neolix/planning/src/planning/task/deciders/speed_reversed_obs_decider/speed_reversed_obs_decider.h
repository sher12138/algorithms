#pragma once

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedReversedObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedReversedObsDecider);

 public:
  virtual ~SpeedReversedObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool InitSingleRoadBoundaries(TaskInfo& task_info);
  bool InitRoadBoundaries(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);

 private:
  void GetSpeedLimitByReversedObs(TaskInfo& task_info);
  void GetSpeedLimitByRepulsiveFieldForReversedObs(TaskInfo& task_info);
  void CalcSpeedLimitbyRepulsiveField(TaskInfo& task_info);
  void SetupReverseSafeIntervalSpeedLimit(TaskInfo& task_info);
  double CalDecelerationByObs(const double lon_dis,
                              const double speed_at_slowdown_to_emergeny);
  double GetSpeedAtEmergencyPoint(const double heading_diff,
                                  const Obstacle& obs, const double lon_dis,
                                  const double lat_dis);
  void CheckBeforeComeInSingleRoad(
      const Obstacle& obstacle,
      std::unordered_set<int>& dp_st_map_ignore_dynamic_obs_id);
  void AdcSlowDown(double distance_to_stop);
  void ReversedObsContextInfo(const Obstacle& obstacle);
  void SpeedReversedObsContextInfo(
      const std::vector<Boundary>& adc_front_road_boundaries,
      const std::unordered_set<int>& dp_st_map_ignore_dynamic_obs_id);

 private:
  double adc_start_l_{0.0};
  double adc_end_l_{0.0};
  double adc_width_{0.0};
  double adc_length_{0.0};
  double adc_back_edge_to_center_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_s_{0.0};
  double adc_current_v_{0.0};
  double emergency_s_range_{0.0};
  double slow_down_s_range_{0.0};
  std::vector<Boundary> single_road_boundaries_{};
  std::vector<Boundary> check_road_boundaries_{};
  double single_road_width_{std::numeric_limits<double>::infinity()};
  double single_road_pre_check_range_{0.0};
  bool update_limited_speed_{false};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double all_delay_time_{2.0};  // from perception to vehicle's delay time
  double effect_dis{0.0};
  ReversedObsData reversed_obs_data_{};
  std::vector<DynamicObsCautionInfo>* dynamic_obs_caution_infos_ptr{nullptr};

 private:
  std::vector<std::vector<double>> acc_cmd_vecor_{};
  std::vector<std::vector<double>> ego_speed_vector_{};
  std::vector<std::vector<double>> relative_distance_vector_{};
  std::vector<std::vector<double>> ego_position_vector_{};
  std::vector<ReverseCautionObs> reverse_use_repulsive_obs_{};

 private:
  const double kApproximateEqualZero{0.0001};
  const double kSingleRoadEpsilon{0.1};
  const double kHalf{0.5};
  const double kStopWaitBufferEpsilon{0.3};
  const double kPlanningCycleTime{0.1};
  const double kApproximateEqualToZero{0.1};
  const double kRatio{1.21};
  const double kNoLimitSpeed{100.0};
  const double kNoCollisionLatDisSafeBuffer{3};
};

REGISTER_SCENARIO_TASK(SpeedReversedObsDecider);

}  // namespace planning
}  // namespace neodrive
