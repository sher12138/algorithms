#pragma once

#include "motorway_speed_limit_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitReferenceLine final
    : public MotorwaySpeedLimitInterface {
 public:
  MotorwaySpeedLimitReferenceLine();
  virtual ~MotorwaySpeedLimitReferenceLine() = default;

  void ComputeSpeedLimit(TaskInfo& task_info) override;

 private:
  void ReferenceLineLimit(TaskInfo& task_info);

  void JunctionLimit(TaskInfo& task_info);

  void JunctionDynamicLimit(TaskInfo& task_info);

  void SpeedBumpLimit(TaskInfo& task_info);

  bool DynamicSpeedLimitCheck(TaskInfo& task_info);
};

}  // namespace planning
}  // namespace neodrive
