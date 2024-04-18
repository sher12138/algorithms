#pragma once

#include "speed_limit_interface.h"

namespace neodrive {
namespace planning {

class SpeedLimitPath final : public SpeedLimitInterface {
 public:
  SpeedLimitPath();
  virtual ~SpeedLimitPath() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;

 private:
  void KappaLimit(TaskInfo& task_info);

  void DistanceToRoadLimit(TaskInfo& task_info);

  void DistanceToStaticObsLimit(TaskInfo& task_info);

  void LaneBorrowLimit(TaskInfo& task_info);

 private:
  double valid_region_s_{0.};
};

}  // namespace planning
}  // namespace neodrive
