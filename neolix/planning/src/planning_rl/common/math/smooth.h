/**
 * @file smooth.h
 **/

#pragma once
#include <array>
#include <cmath>
#include <vector>
#include "common/planning_map/planning_map.h"
#include "vec2d.h"

namespace neodrive {
namespace planning_rl {

class Smooth {
 public:
  Smooth();

  static Smooth& Instance() {
    static Smooth instance;
    return instance;
  }

  bool linear_interpolate(std::pair<double, double>& start_point,
                          std::pair<double, double>& end_point,
                          std::vector<std::pair<double, double>>* new_points,
                          const double resolution);

  bool linear_interpolate(PlanningRLMap::MapPoint& start_point,
                          PlanningRLMap::MapPoint& end_point,
                          PlanningRLMap::MapPoint2d& new_points,
                          const double resolution);

  PlanningRLMap::MapPoint2d line_linear_interpolate(
      PlanningRLMap::MapPoint2d& path);

  bool smooth_path(std::vector<std::vector<double>>& path,
                   std::vector<std::vector<double>>& new_points,
                   double weight_data, double weight_smooth, double tolerance);

  bool smooth_path(PlanningRLMap::MapPoint2d& path,
                   PlanningRLMap::MapPoint2d& new_points, double weight_data,
                   double weight_smooth, double tolerance);
};

}  // namespace planning_rl
}  // namespace neodrive