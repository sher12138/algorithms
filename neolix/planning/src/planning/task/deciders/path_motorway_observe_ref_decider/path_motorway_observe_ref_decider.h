#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class PathMotorwayObserveRefDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathMotorwayObserveRefDecider);

 public:
  virtual ~PathMotorwayObserveRefDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(PathMotorwayObserveRefDecider);

}  // namespace planning
}  // namespace neodrive
