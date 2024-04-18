#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class PathObserveRefDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathObserveRefDecider);

 public:
  virtual ~PathObserveRefDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(PathObserveRefDecider);

}  // namespace planning
}  // namespace neodrive
