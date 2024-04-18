#pragma once

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/math/back_decision_model/conflict_area_rush_model.h"
#include "src/planning/math/back_decision_model/conflict_area_yield_model.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_data_base_map.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_data_base_path.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_data_base_prediction.h"

namespace neodrive {
namespace planning {
class SimConflictDecider {
  DECLARE_SINGLETON(SimConflictDecider);

 public:
  SimConflictDecider(const auto& c_agents) : conflict_infos_(c_agents) {}
  ~SimConflictDecider() = default;
  bool Execute(TaskInfo& task_info);

 private:
  bool CalcConflictArea(TaskInfo& task_info,
                        std::vector<ConnectionConflictInfo>& conflict_info);
  bool Init(TaskInfo& task_info);
  bool CalInputData(TaskInfo& task_info);
  std::vector<std::vector<ConnectionConflictInfo>> CombineConflictInfos(
      std::vector<ConnectionConflictInfo>& conflict_infos);
  bool SplitFRAreaBound();
  bool CalPostProcessDataForSkip(TaskInfo& task_info);
  void SubmitSpeedDecision(TaskInfo& task_info);
  bool CalInteractiveAgent(TaskInfo& task_info);
  bool SetRushLimit(TaskInfo& task_info);
  bool IsRedecision(const double rear_bound, const double constraints_time);
  bool SetYieldLimit(TaskInfo& task_info);
  bool CalPostProcessData(TaskInfo& task_info,
                          const MergeAreaEgoStateSequence& pre_data,
                          bool is_yield, std::vector<double>* s_set,
                          std::vector<double>* v_set,
                          std::vector<double>* a_set);
  bool CalRushProbByGameTheory(const ConnectionConflictInfo& c,
                               const std::vector<double>& sig_bounds,
                               double* rush_prob_diff);

 private:
  std::vector<IntegrativeConflictInfo> conflict_infos_;
  std::vector<std::vector<ConnectionConflictInfo>> process_data_;
  std::vector<std::vector<double>> sigment_fr_bounds_;
  std::vector<double> conflict_area_bound_, conflict_area_rush_bound_,
      conflict_area_yield_bound_;
  VehicleInfo ego_;
  ConnectionConflictInfo key_conflict_info_;
  double risk_prob_;
  std::vector<VehicleInfo> other_agents_;
  std::vector<double> speed_limit_, s_limit_, upstream_speed_limit_,
      upstream_s_limit_, upstream_a_limit_;
  double special_t_;
  double time_step_ = 0.1;
  MergeAreaEgoStateSequence expected_state_;
  MergeAreaEgoStateSequence input_iter_data_;
  MotorwaySequenceDecisionData goal_decision_{
      .source = MotorwaySequenceDecisionType::NONE};
  const config::AutoPlanningResearchConfig::SpeedVehicleBackCipvDeciderConfig*
      motorway_speed_back_cipv_decider_config_ptr_{nullptr};
};
}  // namespace planning
}  // namespace neodrive