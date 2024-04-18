#pragma once

#include <string>
#include <vector>

#include "common/macros.h"
#include "planning.pb.h"

namespace neodrive {
namespace planning {

class BehaviorSpeedLimit {
 public:
  BehaviorSpeedLimit();
  void Reset();

  void SetSpeedLimit(const neodrive::global::planning::SpeedLimit& speed_limit);

  std::string GetSpeedLimitResStr() const;

  std::string AggregateSpeedLimitStr() const;

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(double, speed_limit);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::string, speed_limit_type);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, drive_strategy_max_speed);

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<bool>,
                                             speed_limits_enable);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<neodrive::global::planning::SpeedLimit>, speed_limits);

 private:
  void ComputeSpeedLimit();

 private:
  std::vector<neodrive::global::planning::SpeedLimit> speed_limits_;
  std::vector<bool> speed_limits_enable_;

  double speed_limit_;
  std::string speed_limit_type_{"SPEED_LIMIT"};

  double drive_strategy_max_speed_{0.};
};

}  // namespace planning
}  // namespace neodrive
