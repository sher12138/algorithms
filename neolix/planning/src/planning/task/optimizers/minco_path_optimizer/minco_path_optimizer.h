#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MincoPathOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MincoPathOptimizer);

 public:
  virtual ~MincoPathOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(MincoPathOptimizer);

}  // namespace planning
}  // namespace neodrive
