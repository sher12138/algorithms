#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedGhostStaticObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedGhostStaticObsDecider);

 public:
  virtual ~MotorwaySpeedGhostStaticObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedGhostStaticObsDecider);

}  // namespace planning
}  // namespace neodrive
