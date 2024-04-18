#include "geometry_util.h"

namespace neodrive {
namespace planning {

double GeometryUtil::get_center_shift(double front_edge_to_center,
                                      double back_edge_to_center) {
  return (front_edge_to_center - back_edge_to_center) * 0.5;
}

Vec2d GeometryUtil::get_car_center(double x, double y, double heading,
                                   double center_shift, double lateral_offset) {
  const auto angle = Angle16::from_rad(heading);
  const double sin_angle = sin(angle);
  const double cos_angle = cos(angle);

  // calculate center x
  double c_x = x - sin_angle * lateral_offset + cos_angle * center_shift;
  double c_y = y + cos_angle * lateral_offset + sin_angle * center_shift;
  Vec2d center(c_x, c_y);
  return center;
}

// unbound heading in rad, make it change continuously
bool GeometryUtil::unbound_heading(const std::vector<double>& data_in,
                                   std::vector<double>* data_out) {
  if (data_in.size() < 2) {
    return false;
  }
  data_out->clear();
  data_out->push_back(data_in.front());
  double heading_offset = 0.0;
  for (std::size_t i = 1; i < data_in.size(); ++i) {
    double heading_diff = data_in.at(i) - data_in.at(i - 1);
    if (heading_diff > M_PI) {
      heading_offset -= 2.0 * M_PI;
    } else if (heading_diff < -M_PI) {
      heading_offset += 2.0 * M_PI;
    }
    data_out->push_back(data_in.at(i) + heading_offset);
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
