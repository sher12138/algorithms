#pragma once

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/math/back_decision_model/conflict_area_rush_model.h"
#include "src/planning/math/back_decision_model/conflict_area_yield_model.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedConflictDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedConflictDecider);

 public:
  virtual ~MotorwaySpeedConflictDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  void set_conflict_area_bound(const std::vector<double>& conflict_area_bound) {
    conflict_area_bound_ = conflict_area_bound;
  }

  bool set_speed_limit(const std::vector<double>& speed_limit);

  bool set_s_limit() { return true; };

  bool set_exact_limit();

  void set_speed_limit(const double const_speed_limit_value) {
    speed_limit_ = std::vector<double>(std::floor(special_t_ / time_step_),
                                       const_speed_limit_value);
  }
  bool set_yield_limit(TaskInfo& task_info);

  bool set_rush_limit(TaskInfo& task_info);

 private:
  bool Init(TaskInfo& task_info);

  std::vector<std::vector<ConnectionConflictInfo>> CombineConflictInfos(
      std::vector<ConnectionConflictInfo>& conflict_infos);

  bool CalPostProcessData(TaskInfo& task_info,
                          const MergeAreaEgoStateSequence& pre_data,
                          bool is_yield, std::vector<double>* s_set,
                          std::vector<double>* v_set,
                          std::vector<double>* a_set);
  bool CalPostProcessDataForSkip(TaskInfo& task_info);
  bool CalInteractiveAgent(TaskInfo& task_info);

  bool CalRushProbByGameTheory(const ConnectionConflictInfo& c,
                               const std::vector<double>& sig_bounds,
                               double* rush_prob_diff);
  bool SplitFRAreaBound();
  void SubmitSpeedDecision(TaskInfo& task_info);

  bool FilterFrontDynamicObs(TaskInfo& task_info, const int obs_id);
  bool is_redecision(const double rear_bound, const double constraints_time);
  bool CalInputData(TaskInfo& task_info);
  bool SteerInitialFilter(std::vector<ConnectionConflictInfo>& conflict_info);

 private:
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
  std::vector<std::vector<ConnectionConflictInfo>> process_data_;
  std::vector<std::vector<double>> sigment_fr_bounds_;
  MotorwaySequenceDecisionData goal_decision_{
      .source = MotorwaySequenceDecisionType::NONE};

  const config::AutoPlanningResearchConfig::SpeedVehicleBackCipvDeciderConfig*
      motorway_speed_back_cipv_decider_config_ptr_{nullptr};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedConflictDecider);

}  // namespace planning
}  // namespace neodrive
