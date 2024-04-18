#include "accumulated_s_position_limit.h"

#include "src/planning/common/math/segment2d.h"

namespace neodrive {
namespace planning {

namespace {

using AD4 = std::array<double, 4>;  // s, v, a, t
using PAD = std::pair<std::vector<AD4>, bool>;

std::vector<PAD> GenerateSTGrids(double init_speed, double total_time,
                                 double max_speed, double max_deceleration,
                                 double max_acceleration, double a_resolution,
                                 double t_resolution) {
  std::vector<PAD> ans{};
  for (double a = max_acceleration; a >= max_acceleration; a -= a_resolution) {
    std::vector<AD4> pts{};
    double v{init_speed}, s{0.};
    for (double t = 0.; t < total_time; t += t_resolution) {
      v = std::max(0., init_speed + a * t);
      s = v >= 0. ? (init_speed * t + 0.5 * a * t * t) : s;
      s = std::min(max_speed * t, s);
      v = std::min(max_speed, v);
      pts.push_back({s, v, a, t});
    }
    ans.emplace_back(std::move(pts), true);
  }

  return ans;
}

// Binary search left bound (a[l] >= b)
int LbsIdx(const std::vector<double>& a, const double b) {
  int m = static_cast<int>(a.size());
  if (b < a[0]) return 0;
  if (b > a[m - 1]) return m - 1;
  int l = 0, r = m - 1;
  while (l < r) {
    int mid = l + (r - l) / 2;
    if (a[mid] < b) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }
  return l;
}

void CollisionCheck(const STGraphBoundary& st_boundary,
                    const std::vector<double>& time_vec, double t_l, double t_r,
                    std::vector<PAD>& st_grids) {
  auto l_t_index = LbsIdx(time_vec, t_l), r_t_index = LbsIdx(time_vec, t_r);
  auto l_t_front_index = l_t_index > 0 ? l_t_index - 1 : l_t_index;
  auto l_t_back_index =
      l_t_index + 1 < time_vec.size() ? l_t_index + 1 : l_t_index;
  auto r_t_front_index = r_t_index > 0 ? r_t_index - 1 : r_t_index;
  auto r_t_back_index =
      r_t_index + 1 < time_vec.size() ? r_t_index + 1 : r_t_index;
  for (auto& [st_points, flag] : st_grids) {
    if (flag == false) continue;
    auto [s1, v1, a1, t1] = st_points[l_t_front_index];
    auto [s2, v2, a2, t2] = st_points[l_t_back_index];
    auto [s3, v3, a3, t3] = st_points[r_t_front_index];
    auto [s4, v4, a4, t4] = st_points[r_t_back_index];
    Segment2d segment1({s1, t1}, {s2, t2}), segment2({s2, t2}, {s3, t3}),
        segment3({s3, t3}, {s4, t4});
    if (st_boundary.has_overlap(segment1) ||
        st_boundary.has_overlap(segment2) ||
        st_boundary.has_overlap(segment3)) {
      flag = false;
      break;
    }
  }
}

std::vector<std::size_t> SearchValidSTGrids(
    const std::vector<STGraphBoundary>& st_graph_boundaries,
    std::vector<PAD>& st_grids) {
  std::vector<std::size_t> ans{};
  if (st_grids.empty()) return ans;

  std::vector<double> time_vec{};
  for (auto& [s, v, a, t] : st_grids[0].first) {
    time_vec.push_back(t);
  }
  for (const auto& st_boundary : st_graph_boundaries) {
    CollisionCheck(st_boundary, time_vec, st_boundary.min_t(),
                   st_boundary.max_t(), st_grids);
  }
  for (std::size_t i = 0; i < st_grids.size(); ++i) {
    auto& [st_pts, flag] = st_grids[i];
    if (flag) ans.push_back(i);
  }

  return ans;
}

double ComputeSpeedLimit(
    const std::vector<STGraphBoundary>& st_graph_boundaries,
    std::vector<PAD>& st_grids, std::vector<std::size_t>& valid_indexes,
    double max_speed, double preview_time) {
  double speed_limit{0.};

  /// Speed Limit with min_t of st_graph_boundaries
  if (valid_indexes.empty()) {
  }

  /// Add cost with t->s(overtake/yield distance)

  return speed_limit;
}

}  // namespace

double AccumulatedSPositionLimit::ComputeLimit(
    const std::vector<STGraphBoundary>& st_graph_boundaries,
    const double init_speed, const double max_speed, const double total_time,
    const double max_deceleration, const double max_acceleration,
    const double a_resolution, const double t_resolution,
    const double preview_time) {
  if (st_graph_boundaries.empty()) return max_speed;

  /// Generate s-t grid with const acceleration/deceleration
  auto st_grids =
      GenerateSTGrids(init_speed, total_time, max_speed, max_deceleration,
                      max_acceleration, a_resolution, t_resolution);

  /// Search valid s-t points
  auto valid_indexes = SearchValidSTGrids(st_graph_boundaries, st_grids);

  /// Compute speed limit
  return ComputeSpeedLimit(st_graph_boundaries, st_grids, valid_indexes,
                           max_speed, preview_time);
}

}  // namespace planning
}  // namespace neodrive
