#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedVirtualObsPreDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedVirtualObsPreDecider);

 public:
  virtual ~MotorwaySpeedVirtualObsPreDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool VirtualObsPreDecision(const ReferenceLinePtr reference_line,
                             const InsidePlannerData& inside_data,
                             const std::vector<Obstacle*>& static_obs_vec,
                             OutsidePlannerData* const outside_data) const;

  bool CollisionCheckVirtualObstacle(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data) const;
};

REGISTER_SCENARIO_TASK(MotorwaySpeedVirtualObsPreDecider);

}  // namespace planning
}  // namespace neodrive
