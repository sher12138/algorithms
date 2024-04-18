#include "src/planning/math/curve_fit/polygon_curve_fit.h"

#include <algorithm>

#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

namespace {

using AD2 = std::array<double, 2>;

// https://en.wikipedia.org/wiki/Convex_hull_algorithms
// Andrew's monotone chain convex hull algorithm
std::vector<std::array<double, 2>> AndrewConvexHull(std::vector<AD2> pts) {
  if (pts.size() < 3) return {};

  LOG_INFO("size {}", pts.size());

  std::sort(pts.begin(), pts.end(),
            [](auto& a, auto& b) { return a[0] < b[0]; });
  auto cross = [](auto& p1, auto& p2, auto& p0) {
    return (p1[0] - p0[0]) * (p2[1] - p0[1]) -
           (p1[1] - p0[1]) * (p2[0] - p0[0]);
  };

  int m = pts.size();
  std::vector<int> hull{};
  std::vector<bool> v(m, true);

  /// Upper bound
  for (int i = 0; i < m; ++i) {
    while (hull.size() > 1 &&
           cross(pts[hull.back()], pts[i], pts[*(hull.rbegin() + 1)]) > 0) {
      v[hull.back()] = false;
      hull.pop_back();
    }
    hull.push_back(i);
  }

  /// Lower bound
  v[0] = false;
  for (int i = m - 1; i >= 0; --i) {
    if (v[i]) {
      continue;
    }
    while (hull.size() > 1 &&
           cross(pts[hull.back()], pts[i], pts[*(hull.rbegin() + 1)]) > 0) {
      v[hull.back()] = false;
      hull.pop_back();
    }
    hull.push_back(i);
  }
  hull.pop_back();
  for (auto i : hull) {
    LOG_DEBUG("hull idx {}", i);
  }

  std::vector<AD2> ans{};
  for (auto i : hull) {
    ans.push_back(pts[i]);
  }

  return ans;
}

}  // namespace

PolygonCurveFit::PolygonCurveFit(const std::vector<AD2>& pts)
    : hull_{AndrewConvexHull(pts)} {
  using namespace std;
  left_idx_ = distance(hull_.begin(), min_element(hull_.begin(), hull_.end()));
  right_idx_ = distance(hull_.begin(), max_element(hull_.begin(), hull_.end()));

  auto gt = [](auto& a, auto& b) { return a[1] < b[1]; };
  upper_idx_ =
      distance(hull_.begin(), min_element(hull_.begin(), hull_.end(), gt));
  lower_idx_ =
      distance(hull_.begin(), max_element(hull_.begin(), hull_.end(), gt));
  LOG_INFO("left {}, right {}, up {}, low {}", left_idx_, right_idx_,
           upper_idx_, lower_idx_);
}

std::vector<std::array<double, 2>> PolygonCurveFit::GetLowerCurve() const {
  std::vector<std::array<double, 2>> ans{};
  if (hull_.empty()) return ans;

  int idx = right_idx_;
  while (idx != left_idx_) {
    ans.push_back(hull_[idx]);
    idx = ++idx % hull_.size();
  }
  ans.push_back(hull_[idx]);

  return ans;
}

std::vector<std::array<double, 2>> PolygonCurveFit::GetUpperCurve() const {
  std::vector<std::array<double, 2>> ans{};
  if (hull_.empty()) return ans;

  auto idx = left_idx_;
  while (idx != right_idx_) {
    ans.push_back(hull_[idx]);
    idx = ++idx % hull_.size();
  }
  ans.push_back(hull_[idx]);

  return ans;
}

std::vector<std::array<double, 2>> PolygonCurveFit::GetLeftCurve() const {
  std::vector<std::array<double, 2>> ans{};
  if (hull_.empty()) return ans;

  auto idx = lower_idx_;
  while (idx != upper_idx_) {
    ans.push_back(hull_[idx]);
    idx = ++idx % hull_.size();
  }
  ans.push_back(hull_[idx]);

  return ans;
}

std::vector<std::array<double, 2>> PolygonCurveFit::GetRightCurve() const {
  std::vector<std::array<double, 2>> ans{};
  if (hull_.empty()) return ans;

  int idx = upper_idx_;
  while (idx != lower_idx_) {
    ans.push_back(hull_[idx]);
    idx = ++idx % hull_.size();
  }
  ans.push_back(hull_[idx]);

  return ans;
}

}  // namespace planning
}  // namespace neodrive
