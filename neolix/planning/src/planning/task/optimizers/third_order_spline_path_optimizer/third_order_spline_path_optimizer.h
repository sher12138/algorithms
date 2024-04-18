#pragma once

#include <string>
#include <vector>

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ThirdOrderSplinePathOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ThirdOrderSplinePathOptimizer);

 public:
  virtual ~ThirdOrderSplinePathOptimizer() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};
REGISTER_SCENARIO_TASK(ThirdOrderSplinePathOptimizer);
}  // namespace planning
}  // namespace neodrive
