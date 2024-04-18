#pragma once

#include <vector>

#include "common/math/angle.h"
#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

class GeometryUtil {
 public:
  GeometryUtil() = delete;
  ~GeometryUtil() = delete;

  static double get_center_shift(double front_edge_to_center,
                                 double back_edge_to_center);

  static Vec2d get_car_center(double x, double y, double heading,
                              double center_shift, double lateral_offset);

  static bool unbound_heading(const std::vector<double>& data_in,
                              std::vector<double>* data_out);
};

}  // namespace planning
}  // namespace neodrive
