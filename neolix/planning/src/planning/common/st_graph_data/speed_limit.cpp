#include "speed_limit.h"

#include <algorithm>

#include "common/math/double.h"

namespace neodrive {
namespace planning {

SpeedLimit::~SpeedLimit() { speed_point_.clear(); }

std::vector<SpeedPoint>* SpeedLimit::mutable_speed_limits() {
  return &speed_point_;
}

const std::vector<SpeedPoint>& SpeedLimit::SpeedLimits() const {
  return speed_point_;
}

double SpeedLimit::GetSpeedLimit(const double s) const {
  double ref_s = s;
  if (speed_point_.size() == 0) {
    return 0.0;
  }

  if (speed_point_.size() == 1) {
    return speed_point_.front().v();
  }

  if (ref_s > speed_point_.back().s()) {
    return speed_point_.back().v();
  }

  if (ref_s < speed_point_.front().s()) {
    return speed_point_.front().v();
  }

  auto func = [](const SpeedPoint& sp, const double s) { return sp.s() < s; };

  auto it_lower =
      std::lower_bound(speed_point_.begin(), speed_point_.end(), s, func);
  if (it_lower == speed_point_.begin()) {
    return speed_point_.front().v();
  }

  double weight = 0.0;
  double range = (*it_lower).s() - (*(it_lower - 1)).s();
  if (Double::compare(range, 0.0) > 0) {
    weight = (s - (*(it_lower - 1)).s()) / range;
  }
  return SpeedPoint::interpolate(*(it_lower - 1), *it_lower, weight).v();
}

std::vector<double> SpeedLimit::ValueSpeedLimits() const {
  std::vector<double> ret(speed_point_.size(), 0.0);
  for (std::size_t i = 0; i < speed_point_.size(); ++i) {
    ret[i] = speed_point_[i].v();
  }
  return ret;
}

}  // namespace planning
}  // namespace neodrive
