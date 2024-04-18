#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace neodrive {
namespace planning {

class OccupyMap {
 public:
  using AD2 = std::array<double, 2>;
  using AD3 = std::array<double, 3>;
  using AI2 = std::array<int, 2>;
  struct Config {
    double grid_step{0.1};

    double rectangle_half_width{0.54};
    double rectangle_front_half_length{2.225};
    double rectangle_back_half_length{0.465};
    double rectangle_heading_step{5. / 180. * M_PI};
  };

 public:
  /// Must init with params
  OccupyMap() = delete;
  /// For return value optimization
  OccupyMap(OccupyMap&&) = default;
  OccupyMap& operator=(OccupyMap&&) = default;

  /// Constructor
  /// @param conf Configure variables
  /// @param polygon Polygon to determine the grid boundary, can be nonconvex
  /// @param holes Holes on the polygon, can be nonconvex
  OccupyMap(Config&& conf, const std::vector<AD2>& polygon,
            const std::vector<std::vector<AD2>>& holes);

  /// Interface to check if a point is out of polygon or in holes
  /// @param pt Point to be checked
  /// @return true if the point is out of polygon or in the holes
  bool IsPointOccupied(const AD2& pt) const;

  /// Interface to check if an aabox is out of polygon or in holes
  /// @param lb Low bottom of the aabox
  /// @param rt Right top of the aabox
  /// @return true if the point is out of polygon or in the holes
  bool IsAaBoxOccupied(const AD2& lb, const AD2& rt) const;

  /// Interface to check if vehicle is out of polygon or in holes
  /// @param p3 3D Point(x, y, theta) to be checked
  /// @return true if the rectangle is out of polygon or in the holes
  bool IsVehicleBoxOccupied(const AD3& p3) const;

  bool IsVehicleBoxOccupiedVis(const AD3& p3, const std::string& name) const;

  double GetPointMinDistanceToObstacle(const AD2& pt) const;
  AI2 GetPointGradientDirection(const AD2& pt) const;

  double grid_step() const;

  /// Return range of map
  /// @return [x_min, y_min, x_max, y_max]
  std::array<double, 4> GetRange() const;

  void VisMap() const;

 private:
  Config conf_{};

 private:
  double x_min_{std::numeric_limits<double>::infinity()};
  double x_max_{-std::numeric_limits<double>::infinity()};
  double y_min_{std::numeric_limits<double>::infinity()};
  double y_max_{-std::numeric_limits<double>::infinity()};

  std::vector<std::vector<int>> grid_{};
  std::vector<std::vector<double>> esdf_{};

  struct RectangleIndex {
    int x_min{std::numeric_limits<int>::infinity()};
    int x_max{-std::numeric_limits<int>::infinity()};
    int y_min{std::numeric_limits<int>::infinity()};
    int y_max{-std::numeric_limits<int>::infinity()};

    std::vector<std::array<int, 3>> row_range{};
  };
  std::vector<RectangleIndex> rec_lib_{};
};

}  // namespace planning
}  // namespace neodrive
