#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedStaticObsPreDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedStaticObsPreDecider);

 public:
  virtual ~MotorwaySpeedStaticObsPreDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);

  ErrorCode StaticObstaclePreDecision(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data,
      const std::vector<Obstacle*>& dynamic_obs_vec,
      OutsidePlannerData* const outside_data) const;

  bool IsStaticObsNeedIgnore(const SLPoint& vel_sl,
                             const Boundary& obs_boundary,
                             const double& left_road_bound,
                             const double& right_road_bound,
                             const bool is_forward, bool& is_valid) const;

  bool FreespaceFilter(const std::vector<Boundary>& adc_sl_boundaries,
                       const SLPoint& vel_sl, const Boundary& obs_boundary,
                       const bool id_forward, bool& is_valid) const;

  ErrorCode CollisionCheckObstacleWithoutTrajectory(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data) const;
};

REGISTER_SCENARIO_TASK(MotorwaySpeedStaticObsPreDecider);

}  // namespace planning
}  // namespace neodrive
