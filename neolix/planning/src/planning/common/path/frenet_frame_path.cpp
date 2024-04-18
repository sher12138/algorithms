#include "frenet_frame_path.h"

#include "common/math/interpolation.h"

namespace neodrive {
namespace planning {

FrenetFramePath::FrenetFramePath(
    const std::vector<FrenetFramePoint>& sl_points) {
  points_ = sl_points;
}

void FrenetFramePath::set_frenet_points(
    const std::vector<FrenetFramePoint>& points) {
  points_ = points;
}

std::vector<FrenetFramePoint>* FrenetFramePath::mutable_points() {
  return &points_;
}

const std::vector<FrenetFramePoint>& FrenetFramePath::points() const {
  return points_;
}

std::size_t FrenetFramePath::num_of_points() const { return points_.size(); }

bool FrenetFramePath::point_at(const std::size_t index,
                               FrenetFramePoint& pt) const {
  if (points_.empty()) return false;

  if (index >= points_.size()) {
    pt = points_.back();
  } else {
    pt = points_[index];
  }
  return true;
}

bool FrenetFramePath::interpolate(const double s, FrenetFramePoint& pt) const {
  if (points_.empty()) return false;

  if (s > points_.back().s() + 1.0e-6) {
    pt = points_.back();
    return true;
  }
  if (s < points_.front().s() - 1.0e-6) {
    pt = points_.front();
    return true;
  }

  auto func = [](const FrenetFramePoint& p, const double s) {
    return p.s() < s;
  };

  auto it_lower = std::lower_bound(points_.begin(), points_.end(), s, func);
  if (it_lower == points_.begin()) {
    pt = points_.front();
    return true;
  } else if (it_lower == points_.end()) {
    pt = points_.back();
    return true;
  }
  const auto& p0 = *(it_lower - 1);
  const auto s0 = p0.s();
  const auto& p1 = *it_lower;
  const auto s1 = p1.s();

  double l = Interpolation::lerp(p0.l(), s0, p1.l(), s1, s);
  double dl = Interpolation::lerp(p0.dl(), s0, p1.dl(), s1, s);
  double ddl = Interpolation::lerp(p0.ddl(), s0, p1.ddl(), s1, s);

  FrenetFramePoint tmp_pt(s, l, dl, ddl);
  pt = tmp_pt;
  return true;
}

void FrenetFramePath::clear() { points_.clear(); }

}  // namespace planning
}  // namespace neodrive
