#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedLimitDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedLimitDecider);

 public:
  virtual ~ParkingSpeedLimitDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
};

REGISTER_SCENARIO_TASK(ParkingSpeedLimitDecider);

}  // namespace planning
}  // namespace neodrive