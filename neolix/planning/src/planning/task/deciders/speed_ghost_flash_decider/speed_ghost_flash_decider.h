#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedGhostFlashDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedGhostFlashDecider);

 public:
  virtual ~SpeedGhostFlashDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
};
REGISTER_SCENARIO_TASK(SpeedGhostFlashDecider);
}  // namespace planning
}  // namespace neodrive