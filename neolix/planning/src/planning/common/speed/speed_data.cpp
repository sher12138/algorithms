#include "speed_data.h"

#include "common/math/double.h"

namespace neodrive {
namespace planning {

SpeedData::SpeedData(const std::vector<SpeedPoint> &speed_points)
    : speed_vector_(speed_points) {}

bool SpeedData::build_from_st_points(const std::vector<STPoint> &st_points,
                                     const double init_v, const double init_a,
                                     const double init_jerk) {
  if (st_points.size() < 2) return false;

  double curr_v = init_v;
  double curr_a = init_a;
  double pre_v = curr_v;
  double pre_a = curr_a;

  speed_vector_.emplace_back(
      SpeedPoint(st_points.front(), curr_v, curr_a, init_jerk));

  for (std::size_t i = 1; i < st_points.size(); ++i) {
    double dt = std::max(0.0001, st_points[i].t() - st_points[i - 1].t());
    double average_v = (st_points[i].s() - st_points[i - 1].s()) / dt;
    curr_v = std::max(0.0, 2 * average_v - pre_v);
    if (curr_v < kMathEpsilon) {
      break;
    }
    curr_a = (curr_v - pre_v) / dt;
    double curr_j = (curr_a - pre_a) / dt;
    speed_vector_.emplace_back(
        SpeedPoint(st_points[i], curr_v, curr_a, curr_j));
    pre_v = curr_v;
    pre_a = curr_a;
  }

  return speed_vector_.size() >= 2;
}

bool SpeedData::get_speed_point_with_time(const double t,
                                          SpeedPoint *const speed_point) const {
  if (speed_vector_.size() < 2 || speed_point == nullptr) return false;

  auto it = std::lower_bound(
      speed_vector_.begin(), speed_vector_.end(), t,
      [](const SpeedPoint &sp, double value) { return sp.t() < value; });

  if (it == speed_vector_.end() || std::next(it) == speed_vector_.end()) {
    LOG_ERROR("Error in bounds: t = {:.3f}, speed_vector.size() = {}", t,
              speed_vector_.size());
    return false;
  }

  if (t > it->t() + kMathEpsilon) {
    LOG_ERROR("min_t, max_t, match_t, t: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
              speed_vector_.front().t(), speed_vector_.back().t(), it->t(), t);
    return false;
  }

  // interpolate it and it + 1
  double weight{0.0};
  if (Double::compare(std::next(it)->t(), it->t()) > 0) {
    weight = (t - it->t()) / (std::next(it)->t() - it->t());
  }

  *speed_point = SpeedPoint::interpolate(*it, *std::next(it), weight);
  return true;
}

double SpeedData::total_time() const {
  if (speed_vector_.empty()) return 0.0;

  return speed_vector_.back().t() - speed_vector_.front().t();
}

bool SpeedData::sanity_check() const {
  for (const auto &speed_point : speed_vector()) {
    if (speed_point.s() < 0.0 || speed_point.v() < 0.0) {
      return false;
    }
  }
  return true;
}

void SpeedData::clear() { speed_vector_.clear(); }

}  // namespace planning
}  // namespace neodrive
