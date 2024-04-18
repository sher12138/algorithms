#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class MotorwaySpeedLimitProtectionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedLimitProtectionDecider);

 public:
  virtual ~MotorwaySpeedLimitProtectionDecider() override;

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedLimitProtectionDecider);

}  // namespace planning
}  // namespace neodrive
