#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedLimitProtectionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedLimitProtectionDecider);

 public:
  virtual ~SpeedLimitProtectionDecider() override;

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(SpeedLimitProtectionDecider);

}  // namespace planning
}  // namespace neodrive
