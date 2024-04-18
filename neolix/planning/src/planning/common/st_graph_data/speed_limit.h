#pragma once

#include <vector>

#include "src/planning/common/speed/speed_point.h"

namespace neodrive {
namespace planning {

class SpeedLimit {
 public:
  SpeedLimit() = default;

  ~SpeedLimit();

  std::vector<SpeedPoint>* mutable_speed_limits();

  const std::vector<SpeedPoint>& SpeedLimits() const;

  double GetSpeedLimit(const double s) const;

  std::vector<double> ValueSpeedLimits() const;

 private:
  std::vector<SpeedPoint> speed_point_;
};

}  // namespace planning
}  // namespace neodrive
