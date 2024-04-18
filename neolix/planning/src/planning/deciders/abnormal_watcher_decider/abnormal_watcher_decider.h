#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class AbnormalWatcherDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(AbnormalWatcherDecider);

 public:
  virtual ~AbnormalWatcherDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool trigger_pose_protection_{false};
  bool update_limited_speed_{false};
  double limited_speed_{1.0};
};

REGISTER_SCENARIO_TASK(AbnormalWatcherDecider);

}  // namespace planning
}  // namespace neodrive
