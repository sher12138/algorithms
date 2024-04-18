#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class ParkingSpeedReverseDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ParkingSpeedReverseDecider);

 public:
  virtual ~ParkingSpeedReverseDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  std::vector<Vec2d> GetLidarPoints(
    const neodrive::global::perception::Freespace& freespace) const;
};

REGISTER_SCENARIO_TASK(ParkingSpeedReverseDecider);

}  // namespace planning
}  // namespace neodrive