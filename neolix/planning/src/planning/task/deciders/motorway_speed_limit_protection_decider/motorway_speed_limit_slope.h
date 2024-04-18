#pragma once

#include "motorway_speed_limit_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitSlope final : public MotorwaySpeedLimitInterface {
 public:
  MotorwaySpeedLimitSlope();
  virtual ~MotorwaySpeedLimitSlope() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
