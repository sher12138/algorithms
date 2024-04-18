/// Fit the convex polygon curve with given points
#pragma once

#include <array>
#include <vector>

namespace neodrive {
namespace planning {

/// @class Fit the points with convex hull polygon boundary
class PolygonCurveFit {
 public:
  explicit PolygonCurveFit(const std::vector<std::array<double, 2>>& pts);

  std::vector<std::array<double, 2>> GetLowerCurve() const;
  std::vector<std::array<double, 2>> GetUpperCurve() const;
  std::vector<std::array<double, 2>> GetLeftCurve() const;
  std::vector<std::array<double, 2>> GetRightCurve() const;

 private:
  std::vector<std::array<double, 2>> hull_{};
  int left_idx_{0};
  int right_idx_{0};
  int upper_idx_{0};
  int lower_idx_{0};
};

}  // namespace planning
}  // namespace neodrive
