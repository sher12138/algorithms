#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedStaticObsPreDecisionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedStaticObsPreDecisionDecider);

 public:
  virtual ~SpeedStaticObsPreDecisionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

  ErrorCode VirtualObsPreDecision(TaskInfo& task_info,
                                  const InsidePlannerData& inside_data,
                                  const std::vector<Obstacle*>& static_obs_vec,
                                  OutsidePlannerData* const outside_data) const;

 private:
  bool Process(TaskInfo& task_info);

 private:
  ErrorCode StaticObsPreDecision(TaskInfo& task_info,
                                 const ReferenceLinePtr& reference_line,
                                 const InsidePlannerData& inside_data,
                                 const std::vector<Obstacle*>& static_obs_vec,
                                 OutsidePlannerData* const outside_data) const;

  ErrorCode LowSpeedDynamicObsPreDecision(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const std::vector<Obstacle*>& dynamic_obs_vec,
      OutsidePlannerData* const outside_data) const;

  bool GetRoadBound(const ReferenceLinePtr& reference_line,
                    const InsidePlannerData& inside_data,
                    const Boundary& boundary, double* left_bound,
                    double* right_bound) const;

  ErrorCode CollisionCheckObstacleWithoutTrajectory(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const Obstacle& obstacle, OutsidePlannerData* const outside_data,
      std::vector<SpeedObstacleDecision>* obstacles_decision) const;

  ErrorCode CollisionPreCheckObstacleWithoutTrajectory(
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data,
      std::vector<SpeedObstacleDecision>* obstacles_decision) const;

  bool IsStaticObsNeedIgnore(const SLPoint& vel_sl,
                             const Boundary& obs_boundary,
                             const double& left_road_bound,
                             const double& right_road_bound,
                             const bool is_forward, bool& is_valid) const;

  bool FreespaceFilter(const std::vector<Boundary>& adc_sl_boundaries,
                       const double max_filter_s, const double min_filter_s,
                       const SLPoint& vel_sl, const Boundary& obs_boundary,
                       const bool id_forward, bool& is_valid) const;

  bool IsDynamicObsNeedIgnore(const InsidePlannerData& inside_data,
                              const Obstacle& obstacle, bool& is_valid) const;

  void StaticContextInfo(OutsidePlannerData* const outside_data);
};
REGISTER_SCENARIO_TASK(SpeedStaticObsPreDecisionDecider);
}  // namespace planning
}  // namespace neodrive