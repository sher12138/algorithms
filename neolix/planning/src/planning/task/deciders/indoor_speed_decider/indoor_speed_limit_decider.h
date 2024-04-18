#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class IndoorSpeedLimitDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(IndoorSpeedLimitDecider);

 public:
  virtual ~IndoorSpeedLimitDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
};

REGISTER_SCENARIO_TASK(IndoorSpeedLimitDecider);

}  // namespace planning
}  // namespace neodrive