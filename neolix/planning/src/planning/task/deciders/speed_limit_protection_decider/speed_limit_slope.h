#pragma once

#include "speed_limit_interface.h"

namespace neodrive {
namespace planning {

class SpeedLimitSlope final : public SpeedLimitInterface {
 public:
  SpeedLimitSlope();
  virtual ~SpeedLimitSlope() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
