#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class OutOfDeadEndDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(OutOfDeadEndDecider);
 public:
  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override;
  void Reset() override;
};

REGISTER_SCENARIO_TASK(OutOfDeadEndDecider);

}  // namespace planning
}  // namespace neodrive
