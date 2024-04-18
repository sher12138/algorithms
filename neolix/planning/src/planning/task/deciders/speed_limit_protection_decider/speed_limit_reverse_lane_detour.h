#pragma once

#include "speed_limit_interface.h"

namespace neodrive {
namespace planning {

class SpeedLimitReverseLaneDetour final : public SpeedLimitInterface {
 public:
  SpeedLimitReverseLaneDetour();
  virtual ~SpeedLimitReverseLaneDetour() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
