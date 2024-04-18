#include "src/planning/math/curve_fit/linear_curve_fit.h"

#include <Eigen/Core>

#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double threshold_dx = 0.5;
constexpr double threshold_dy = 0.5;

constexpr std::size_t threshold_pt_size = 5;

using AD2 = std::array<double, 2>;

// y = ax + b, return a and b
AD2 GetLinearParma(const std::vector<AD2>& pts) {
  const std::size_t N = pts.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Constant(N, 2, 1);
  for (std::size_t i = 0; i < N; ++i) A(i, 1) = pts[i][0];
  Eigen::MatrixXd b(N, 1);
  for (std::size_t i = 0; i < N; ++i) b(i, 0) = pts[i][1];

  auto AT = A.transpose();
  auto param = (AT * A).inverse() * AT * b;

  return {param(1, 0), param(0, 0)};
}

AD2 GetProjectPoint(const double a, const double b, const AD2& pt) {
  Vec2d base{0, b};
  auto uv = Vec2d::create_unit_vec(std::atan(a));

  auto vp = Vec2d{pt[0], pt[1]} - base;

  auto ans = uv.inner_prod(vp) * uv + base;

  return {ans.x(), ans.y()};
}

}  // namespace

LinearCurveFit::LinearCurveFit(const std::vector<AD2>& pts) : points_{pts} {}

std::vector<std::array<AD2, 2>> LinearCurveFit::GetLineSegments() const {
  for (auto& [x, y] : points_) {
    LOG_INFO("({}, {})", x, y);
  }
  // cluster
  std::vector<std::vector<AD2>> lines{};
  for (auto& [x1, y1] : points_) {
    if (lines.empty() || std::abs(lines.back().back()[0] - x1) > threshold_dx ||
        std::abs(lines.back().back()[1] - y1) > threshold_dy) {
      lines.push_back({AD2{x1, y1}});
    } else {
      lines.back().push_back({x1, y1});
    }
  }

  for (auto& l : lines) LOG_INFO("line size {}", l.size());

  // remove lines with less points, (maybe) use two pointers to reduce mem usage
  decltype(lines) tmp{};
  for (auto& l : lines) {
    if (l.size() > threshold_pt_size) {
      tmp.push_back(l);
    }
  }

  std::swap(tmp, lines);

  for (auto& l : lines) {
    LOG_INFO("get line: ");
    for (auto& [x, y] : l) {
      LOG_INFO("({}, {})", x, y);
    }
  }

  // fit
  std::vector<std::array<AD2, 2>> ans{};
  for (auto& l : lines) {
    auto [a, b] = GetLinearParma(l);
    LOG_INFO("a b: {}, {}", a, b);
    ans.push_back(
        {GetProjectPoint(a, b, l.front()), GetProjectPoint(a, b, l.back())});
  }

  for (auto& [start, end] : ans) {
    LOG_INFO("ans: ({}, {}) -> ({}, {})", start[0], start[1], end[0], end[1]);
  }

  return ans;
}

std::vector<std::array<double, 2>> LinearCurveFit::GetLineSegment() const {
  auto [a, b] = GetLinearParma(points_);
  auto lower_pt = *std::min_element(points_.begin(), points_.end());
  auto upper_pt = *std::max_element(points_.begin(), points_.end());

  return {GetProjectPoint(a, b, lower_pt), GetProjectPoint(a, b, upper_pt)};
}

}  // namespace planning
}  // namespace neodrive
