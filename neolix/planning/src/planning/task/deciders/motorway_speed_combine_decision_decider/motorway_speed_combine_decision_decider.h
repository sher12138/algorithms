#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedCombineDecisionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedCombineDecisionDecider);

 public:
  virtual ~MotorwaySpeedCombineDecisionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedCombineDecisionDecider);

}  // namespace planning
}  // namespace neodrive
