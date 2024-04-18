#pragma once

#include "motorway_speed_limit_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitTurnRight final : public MotorwaySpeedLimitInterface {
 public:
  MotorwaySpeedLimitTurnRight();
  virtual ~MotorwaySpeedLimitTurnRight() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
