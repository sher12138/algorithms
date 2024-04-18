#pragma once

#include <mutex>
#include <string>

#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

static inline bool CheckPolygon(const PerceptionObstacle &obstacle) {
  static constexpr double kMinArea = 0.001;
  for (auto &point : obstacle.polygon_point()) {
    if (Double::is_nan(point.x()) || Double::is_nan(point.y())) {
      return false;
    }
  }
  if (obstacle.polygon_point_size() < 3) {
    return false;
  }
  double area = 0.0;
  for (int i = 2; i < obstacle.polygon_point_size(); ++i) {
    area += cross_prod(obstacle.polygon_point(0), obstacle.polygon_point(i - 1),
                       obstacle.polygon_point(i));
    if (std::fabs(area) > 2 * kMinArea) {
      return true;
    }
  }
  area = std::fabs(area) / 2.0;
  return !Double::is_nan(area) && area > kMinArea;
}

}  // namespace planning
}  // namespace neodrive