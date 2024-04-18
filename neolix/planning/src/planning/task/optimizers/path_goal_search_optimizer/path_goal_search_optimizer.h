#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class PathGoalSearchOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathGoalSearchOptimizer);

 public:
  virtual ~PathGoalSearchOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(PathGoalSearchOptimizer);

}  // namespace planning
}  // namespace neodrive
