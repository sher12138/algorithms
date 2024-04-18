#pragma once

#include <cmath>
#include <vector>
#include "common/math/vec2d.h"
#include "neolix_common/math/shm_structs/shm_utils.h"
#include "src/planning/navigation/config/navigation_config.h"

namespace neodrive {
namespace common {

/// Calculate angle between current and next vector
/// @param curr current vector
/// @param next next vector
/// @return Angle between current and next vector
static double CalcAngle2NextVec(const math::ShmVector<math::Vec2d>& curr,
                                const math::ShmVector<math::Vec2d>& next) {
  int curr_idx = curr.size() - 1, next_idx = 0;
  const auto& config =
      planning::config::NavigationConfig::Instance()->navigation_config();
  if (curr.size() < 2 || next.size() < 2) return 0.0;
  for (; curr_idx >= 0; --curr_idx) {
    double dx = curr.back().x() - curr[curr_idx].x();
    double dy = curr.back().y() - curr[curr_idx].y();
    double len = sqrt(dx * dx + dy * dy);
    if (len > config.vec_angle_length || curr_idx == 0) break;
  }
  for (; next_idx < next.size(); ++next_idx) {
    double dx = next[0].x() - next[next_idx].x();
    double dy = next[0].y() - next[next_idx].y();
    double len = sqrt(dx * dx + dy * dy);
    if (len > config.vec_angle_length || next_idx == next.size() - 1) break;
  }
  std::vector<double> vec1{curr.back().x() - curr[curr_idx].x(),
                           curr.back().y() - curr[curr_idx].y()};
  std::vector<double> vec2{next[next_idx].x() - next[0].x(),
                           next[next_idx].y() - next[0].y()};
  double dot = vec1[0] * vec2[0] + vec1[1] * vec2[1];
  double len1 = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]);
  double len2 = sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);
  if (len1 == 0 || len2 == 0) return 0.0;
  return acos(dot / (len1 * len2));
}

/// Calculate angle between current and neighbor vector
/// @param curr current vector
/// @param neigh neighbor vector
/// @return Angle between current and neighbor vector
static double CalcAngle2NeighborVec(const math::ShmVector<math::Vec2d>& curr,
                                    const math::ShmVector<math::Vec2d>& neigh) {
  int curr_idx = curr.size() - 1, neigh_idx = neigh.size() - 1;
  const auto& config =
      planning::config::NavigationConfig::Instance()->navigation_config();
  if (curr.size() < 2 || neigh.size() < 2) return 0.0;
  for (; curr_idx >= 0; --curr_idx) {
    double dx = curr.back().x() - curr[curr_idx].x();
    double dy = curr.back().y() - curr[curr_idx].y();
    double len = sqrt(dx * dx + dy * dy);
    if (len > config.vec_angle_length || curr_idx == 0) break;
  }
  for (; neigh_idx >= 0; --neigh_idx) {
    double dx = neigh.back().x() - neigh[neigh_idx].x();
    double dy = neigh.back().y() - neigh[neigh_idx].y();
    double len = sqrt(dx * dx + dy * dy);
    if (len > config.vec_angle_length || neigh_idx == 0) break;
  }
  LOG_DEBUG("curr_idx:{}, neigh_idx:{}", curr_idx, neigh_idx);
  std::vector<double> vec1{curr.back().x() - curr[curr_idx].x(),
                           curr.back().y() - curr[curr_idx].y()};
  std::vector<double> vec2{neigh.back().x() - neigh[neigh_idx].x(),
                           neigh.back().y() - neigh[neigh_idx].y()};
  double dot = vec1[0] * vec2[0] + vec1[1] * vec2[1];
  double len1 = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]);
  double len2 = sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);
  if (len1 == 0 || len2 == 0) return 0.0;
  if (vec1[0] * vec2[1] - vec1[1] * vec2[0] >= 0)
    return acos(dot / (len1 * len2));
  else
    return -acos(dot / (len1 * len2));
}

}  // namespace common
}  // namespace neodrive
