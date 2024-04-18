#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedStaticObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedStaticObsDecider);

 public:
  virtual ~ParkingSpeedStaticObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool StaticObsPreDecision(TaskInfo& task_info) const;
  bool IsStaticObsNeedIgnore(const SLPoint& vel_sl,
                             const Boundary& obs_boundary,
                             const bool is_forward, bool& is_ignore) const;
  ErrorCode CollisionCheckObstacleWithoutTrajectory(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const Obstacle& obstacle, OutsidePlannerData* const outside_data,
      std::vector<SpeedObstacleDecision>* obstacles_decision) const;
};

REGISTER_SCENARIO_TASK(ParkingSpeedStaticObsDecider);

}  // namespace planning
}  // namespace neodrive