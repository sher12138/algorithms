#pragma once

#include "conflict_data_base_map.h"
#include "conflict_data_base_path.h"
#include "conflict_info_interface.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

class ConflictInfoForMergeIn final : public ConflictInfoInterface {
 public:
  ConflictInfoForMergeIn();
  virtual ~ConflictInfoForMergeIn() = default;

  std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) override;

  void JudgeEgoTurnStateAndDirection(TaskInfo& task_info);
  bool ObsIsInTurnDirection(Obstacle* const obs);
  bool EgoScenarioIsCruise(TaskInfo& task_info);

  std::vector<ConnectionConflictInfo> CalMergeConflictInfoBasePath(
      TaskInfo& task_info, std::vector<int>& front_risk_obs);
  std::vector<ConnectionConflictInfo> CalMergeConflictInfoBaseMap(
      TaskInfo& task_info, const std::vector<ConnectionConflictInfo>& cibp_info,
      const std::vector<int>& front_risk_obs);

  bool IgnoreObsOnVehicleFrame(const InsidePlannerData& inside_data,
                               const Obstacle* const obstacle);

  bool FilterWhenInSameLane(Obstacle* const obs, TaskInfo& task_info);

  bool FilterWhenInThirdLane(Obstacle* const obs, TaskInfo& task_info);

  bool JudgeSameLane(Obstacle* const obstacle,
                     const ReferenceLinePtr& reference_line,
                     const double& adc_current_s);

  bool JudgeNecessaryMerge(TaskInfo& task_info, const Obstacle* obs,
                           const uint64_t lane_id, bool& is_necessary);

  void CalcRelativeLatDistance(TaskInfo& task_info, const Obstacle* obs,
                               std::vector<int>& obs_ids);

 private:
  const config::AutoPlanningResearchConfig::
      MotorwaySpeedConfilictDeciderConfig::MergeInRoadScenarioConfig* config_ =
          &config::PlanningConfig::Instance()
               ->planning_research_config()
               .motorway_speed_confilict_decider_config
               .merge_in_road_scenario_config;
  const config::AutoPlanningResearchConfig::SpeedVehicleBackCipvDeciderConfig*
      motorway_speed_back_cipv_decider_config_ptr_ =
          &config::PlanningConfig::Instance()
               ->planning_research_config()
               .speed_vehicle_back_cipv_decider_config;

  std::unordered_set<int> cibp_agents_{};
  std::unordered_set<int> cibm_agents_{};

  bool ego_exist_turn_{false};
  bool ego_is_turn_left_{false};
  double adc_current_s_ = 0.0;
  double adc_current_v_ = 0.0;
  double adc_front_edge_s_ = 0.0;
  double constant_acc_ = motorway_speed_back_cipv_decider_config_ptr_
                             ->constant_mergein_acc_base_map;
};

}  // namespace planning
}  // namespace neodrive
