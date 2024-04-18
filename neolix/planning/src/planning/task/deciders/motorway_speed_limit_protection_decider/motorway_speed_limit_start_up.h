#pragma once

#include "motorway_speed_limit_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitStartUp final : public MotorwaySpeedLimitInterface {
 public:
  MotorwaySpeedLimitStartUp();
  virtual ~MotorwaySpeedLimitStartUp() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
