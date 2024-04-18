#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class FreespacePathOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(FreespacePathOptimizer);

 public:
  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override;

  void Reset() override;
};

REGISTER_SCENARIO_TASK(FreespacePathOptimizer);

}  // namespace planning
}  // namespace neodrive
