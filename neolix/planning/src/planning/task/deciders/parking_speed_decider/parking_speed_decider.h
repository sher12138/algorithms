#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedDecider);

 public:
  virtual ~ParkingSpeedDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  std::vector<Vec2d> GetLidarPoints(
    const neodrive::global::perception::Freespace& freespace) const;
};

REGISTER_SCENARIO_TASK(ParkingSpeedDecider);

}  // namespace planning
}  // namespace neodrive