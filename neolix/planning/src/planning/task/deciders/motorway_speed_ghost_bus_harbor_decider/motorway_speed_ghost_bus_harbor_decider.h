#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedGhostBusHarborDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedGhostBusHarborDecider);

 public:
  virtual ~MotorwaySpeedGhostBusHarborDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedGhostBusHarborDecider);

}  // namespace planning
}  // namespace neodrive
