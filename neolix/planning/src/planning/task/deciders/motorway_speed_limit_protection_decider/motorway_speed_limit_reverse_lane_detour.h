#pragma once

#include "motorway_speed_limit_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitReverseLaneDetour final
    : public MotorwaySpeedLimitInterface {
 public:
  MotorwaySpeedLimitReverseLaneDetour();
  virtual ~MotorwaySpeedLimitReverseLaneDetour() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;
};

}  // namespace planning
}  // namespace neodrive
