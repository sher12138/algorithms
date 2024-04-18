#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedVirtualObsPreDecisionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedVirtualObsPreDecisionDecider);

 public:
  virtual ~SpeedVirtualObsPreDecisionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  void VirtualContextInfo(OutsidePlannerData* const outside_data);
};
REGISTER_SCENARIO_TASK(SpeedVirtualObsPreDecisionDecider);
}  // namespace planning
}  // namespace neodrive
