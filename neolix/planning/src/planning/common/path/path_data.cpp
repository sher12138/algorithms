#include "path_data.h"

namespace neodrive {
namespace planning {

bool PathData::get_path_point_with_path_s(const double s,
                                          PathPoint *const path_point) const {
  if (path_point == nullptr) return false;

  const double epsilon_proximity = 1.0e-1;

  if (!path_.evaluate_linear_approximation(s, *path_point)) {
    return false;
  }

  double param_diff = std::abs(path_point->s() - s);
  if (param_diff > epsilon_proximity) {
    // test
    LOG_ERROR("param_diff {}", param_diff);
    return false;
  }
  return true;
}

void PathData::clear() {
  path_.clear();
  frenet_path_.clear();
}

bool PathData::is_valid_data() const {
  if (path().path_points().size() <= 1) return false;

  for (std::size_t i = 0; i + 1 < path().path_points().size(); ++i) {
    if (path().path_points()[i + 1].s() < path().path_points()[i].s()) {
      LOG_ERROR("path_data not valid-s decrease point[{}]: {}, point[{}]: {}",
                i, path().path_points()[i].s(), i + 1,
                path().path_points()[i + 1].s());
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
