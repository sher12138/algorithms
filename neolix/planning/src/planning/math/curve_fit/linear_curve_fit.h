/// Fit the linear curve with given points
#pragma once

#include <array>
#include <vector>

namespace neodrive {
namespace planning {

/// @class Fit a list of points with linear curve
class LinearCurveFit {
 public:
  explicit LinearCurveFit(const std::vector<std::array<double, 2>>& pts);

  /// Fit to multiple line segments
  /// @return list of [start_point, end_point]
  std::vector<std::array<std::array<double, 2>, 2>> GetLineSegments() const;

  /// Fit to one line segment
  /// @return [start_point, end_point]
  std::vector<std::array<double, 2>> GetLineSegment() const;

 private:
  std::vector<std::array<double, 2>> points_{};
};

}  // namespace planning
}  // namespace neodrive
