#pragma once

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedLimitInterface {
 public:
  explicit MotorwaySpeedLimitInterface(const std::string& name) : name_(name) {}
  virtual ~MotorwaySpeedLimitInterface() = default;

  virtual void ComputeSpeedLimit(TaskInfo& task_info) = 0;

  virtual void SaveSpeedLimit(
      const neodrive::global::planning::SpeedLimit_SourceType& type,
      const neodrive::global::planning::SpeedLimit_ConstraintType&
          constraint_type,
      double upper_bound, double acc) {
    neodrive::global::planning::SpeedLimit new_limit{};
    new_limit.set_source_type(type);
    new_limit.add_upper_bounds(upper_bound);
    new_limit.set_constraint_type(constraint_type);
    new_limit.set_acceleration(acc);
    LOG_DEBUG("{} {} limit speed: speed = {:.2f}, acc = {:.2f}",
              SpeedLimit_SourceType_Name(new_limit.source_type()),
              SpeedLimit_ConstraintType_Name(new_limit.constraint_type()),
              new_limit.upper_bounds().at(0) * 3.6, new_limit.acceleration());

    DataCenter::Instance()->mutable_behavior_speed_limits()->SetSpeedLimit(
        new_limit);
  }

  virtual const std::string& name() const { return name_; }

  virtual const double max_speed() const { return max_speed_; }

 private:
  std::string name_{""};

  double max_speed_ = DataCenter::Instance()->drive_strategy_max_speed();
};

}  // namespace planning
}  // namespace neodrive
