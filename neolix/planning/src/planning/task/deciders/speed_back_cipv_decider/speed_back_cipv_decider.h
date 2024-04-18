#pragma once

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/math/util.h"
#include "src/planning/math/back_decision_model/conflict_area_rush_model.h"
#include "src/planning/math/back_decision_model/conflict_area_yield_model.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/speed_back_cipv_decider/conflict_info_for_merge_in.h"

namespace neodrive {
namespace planning {

class SpeedBackCipvDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedBackCipvDecider);

 public:
  virtual ~SpeedBackCipvDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
  void set_conflict_area_bound(const std::vector<double>& conflict_area_bound) {
    conflict_area_bound_ = conflict_area_bound;
  }
  void set_game_theory_param() {
    double param_a_ = speed_back_cipv_decider_config_ptr_->static_game_theory_a;
    double param_c_ = speed_back_cipv_decider_config_ptr_->static_game_theory_c;
    double param_margin_ =
        speed_back_cipv_decider_config_ptr_->static_game_theory_margin;
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
  bool CalProcessDataBaseMap(TaskInfo& task_info);
  bool CalProcessDataBasePath(
      TaskInfo& task_info,
      const std::vector<ConnectionConflictInfo>& meeting_data);
  bool CalMeetingDataBaseMap(TaskInfo& task_info,
                             std::vector<ConnectionConflictInfo>* meeting_data);
  bool is_merging_area(TaskInfo& task_info);
  bool CalPostProcessData(TaskInfo& task_info,
                          const MergeAreaEgoStateSequence& pre_data,
                          bool is_yield, std::vector<double>* s_set,
                          std::vector<double>* v_set,
                          std::vector<double>* a_set);
  bool CalInteractiveAgent(TaskInfo& task_info);
  bool Init(TaskInfo& task_info);
  bool CalRushProbByGameTheory(const ConnectionConflictInfo& c,
                               const std::vector<double>& sig_bounds,
                               double* rush_prob_diff);
  bool CalOtherAgentBound(const uint64_t merging_lane_id,
                          const common::math::Vec2d& point);
  bool SplitFRAreaBound();
  bool CalAgentInfo(const Obstacle* obs, VehicleInfo* agent_info);
  // rebase guofeng.li
  bool CollisionCheckWithObstaclePolyline(
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data,
      std::vector<double>* conflict_area_bound);
  void GetObsPredictionSegment(const InsidePlannerData& inside_data,
                               const Obstacle& obstacle, OBS_SIDE& obs_side,
                               Segment2d& obs_prediction_segment,
                               Vec2d& obs_base_pt);
  void GetFirstCollideInfo(
      const OutsidePlannerData* const outside_data,
      const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
      AdcCollideCornerPoint& adc_first_collide_corner_point,
      AdcCollideCornerPoint& ego_collide_corner_point,
      AdcCollideCornerPoint& agent_collide_corner_point,
      int& path_first_collide_i, Vec2d& path_first_collide_pt,
      int& path_ego_end_collide_i, Vec2d& path_ego_end_collide_pt,
      int& path_agent_end_collide_i, Vec2d& path_agent_end_collide_pt);
  bool GetLaneMergingInfo(const std::vector<PathPoint>& path_points,
                          const int path_first_collide_i, const int obs_id,
                          const Vec2d& path_first_collide_pt,
                          const int ego_end_collide_i,
                          const int agent_end_collide_i,
                          const Vec2d& agent_end_collide_pt,
                          const double agent_length, const Vec2d& obs_base_pt,
                          std::vector<double>* conflict_area_bound);
  void GetAdcCornerPointCoordinate(
      const Vec2d& adc_coordinate, const double adc_heading,
      std::vector<Vec2d>& adc_corner_pt_coordinate);
  bool is_redecision(TaskInfo& task_info, const double rear_bound,
                     const double constraints_time);
  bool CalInputData(TaskInfo& task_info);
  bool InitialFilter(std::vector<ConnectionConflictInfo>& conflict_info);
  std::vector<std::vector<ConnectionConflictInfo>> CombineConflictInfos(
      std::vector<ConnectionConflictInfo>& conflict_infos);

 private:
  std::vector<double> conflict_area_bound_, conflict_area_rush_bound_,
      conflict_area_yield_bound_;
  VehicleInfo ego_;
  // VehicleInfo key_agent_;
  ConnectionConflictInfo key_conflict_info_;
  double risk_prob_;
  std::vector<VehicleInfo> other_agents_;
  std::vector<double> speed_limit_, s_limit_, upstream_speed_limit_,
      upstream_s_limit_, upstream_a_limit_;
  double special_t_;
  // double param_a_ = 1.0, param_c_ = 1.0, param_margin_ = 0.15;
  double time_step_ = 0.1;
  MergeAreaEgoStateSequence expected_state_;
  MergeAreaEgoStateSequence input_iter_data_;
  std::vector<std::vector<ConnectionConflictInfo>> process_data_;
  std::vector<std::vector<double>> sigment_fr_bounds_;
  const config::AutoPlanningResearchConfig::SpeedVehicleMergingDeciderConfig*
      speed_vehicle_merging_decider_config_ptr_{nullptr};
  const config::AutoPlanningResearchConfig::SpeedVehicleBackCipvDeciderConfig*
      speed_back_cipv_decider_config_ptr_{nullptr};
};

REGISTER_SCENARIO_TASK(SpeedBackCipvDecider);

}  // namespace planning
}  // namespace neodrive
