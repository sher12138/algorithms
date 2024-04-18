#pragma once

#include "speed_limit_interface.h"

namespace neodrive {
namespace planning {

class SpeedLimitStartUp final : public SpeedLimitInterface {
 public:
  SpeedLimitStartUp();
  virtual ~SpeedLimitStartUp() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
