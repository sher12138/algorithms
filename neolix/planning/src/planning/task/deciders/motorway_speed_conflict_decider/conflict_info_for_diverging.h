#pragma once

#include "conflict_data_base_path.h"
#include "conflict_info_interface.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

class ConflictInfoForDiverging final : public ConflictInfoInterface {
 public:
  static ConflictInfoForDiverging& Instance() {
    static ConflictInfoForDiverging instance;
    return instance;
  }

  void Clear();
  void Init(TaskInfo& task_info);
  virtual ~ConflictInfoForDiverging() = default;

  std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) override;

  bool DivergingTriggleJudge(TaskInfo& task_info, const bool ego_in_diverging);

  bool FilterSolution(Obstacle& obs, TaskInfo& task_info);

  void GenerateEgoConsiderDirection(
      TaskInfo& task_info,
      const TrafficConflictZoneContext& traffic_conflict_zone_context,
      const bool ego_in_diverging);

  bool LowerRiskObsIgnore(TaskInfo& task_info, const Obstacle& obstalce);

 private:
  ConflictInfoForDiverging();
  const config::AutoPlanningResearchConfig::
      MotorwaySpeedConfilictDeciderConfig::DivergingRoadScenarioConfig*
          config_ = &config::PlanningConfig::Instance()
                         ->planning_research_config()
                         .motorway_speed_confilict_decider_config
                         .diverging_road_scenario_config;
  // need clear data
  std::unordered_set<int> custom_agents_{};
  double adc_current_s_ = 0.0;
  double adc_current_v_ = 0.0;
  double adc_front_edge_s_ = 0.0;

  // last info
  TrafficConflictZoneContext::connectionType ego_connection_type_ =
      TrafficConflictZoneContext::connectionType::Unknown;
  double keep_length_{0.0};
  double acculate_dis_{0.0};
  Vec2d start_point_{};
  double start_s_{-1.0};
  bool last_is_diverging_{false};
  bool change_sc_sign_{false};
  int ego_consider_direction_{0};
  int history_directory_{0};
  TrafficConflictZoneContext::connectionType last_skip_reason_ =
      TrafficConflictZoneContext::connectionType::Unknown;
};

}  // namespace planning
}  // namespace neodrive
