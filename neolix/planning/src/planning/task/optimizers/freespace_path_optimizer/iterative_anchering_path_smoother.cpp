#include "src/planning/task/optimizers/freespace_path_optimizer/iterative_anchering_path_smoother.h"

#include <cmath>
#include <cstdio>

#include "src/planning/common/math/vec2d.h"
#include "src/planning/task/optimizers/freespace_path_optimizer/freespace_smoother.h"

namespace neodrive {
namespace planning {

namespace {

using AD2 = std::array<double, 2>;
using AD3 = std::array<double, 3>;
using AD4 = std::array<double, 4>;
using Config = IterativeAncheringPathSmoother::Config;

constexpr double kVehHalfWidth{0.54};
constexpr double kVehRearAxeToFront{2.225};
constexpr double kVehRearAxeToBack{0.465};

std::array<AD2, 4> GenerateRectangleEdge(const AD3& pt) {
  Vec2d u{std::cos(pt[2]), std::sin(pt[2])};
  Vec2d pu = u.rotate(M_PI_2);
  Vec2d c{pt[0], pt[1]};

  const double r2f = 2.225, r2b = 0.465, hw = 0.54;
  auto p0 = c + u * r2f + pu * hw;
  auto p1 = c + u * r2f - pu * hw;
  auto p2 = c - u * r2b - pu * hw;
  auto p3 = c - u * r2b + pu * hw;
  return {AD2{p0.x(), p0.y()}, AD2{p1.x(), p1.y()}, AD2{p2.x(), p2.y()},
          AD2{p3.x(), p3.y()}};
}

std::vector<AD4> BuildBounds(const Config& conf, const std::vector<AD3>& pts,
                             const OccupyMap& om, const bool isend) {
  const int n = pts.size();
  const double off = kVehHalfWidth;
  const double s = om.grid_step();
  auto extend = [&](auto x, auto y) {
    int f = 15;
    double lx = 0, hx = 0, ly = 0, hy = 0;
    while (f) {
      if ((f & 1) && lx + hx < conf.half_bound &&
          !om.IsAaBoxOccupied({x - off - lx - s, y - off - ly},
                              {x + off + hx, y + off + hy})) {
        lx += s;
      } else {
        f &= ~1;
      }
      if ((f & 2) && lx + hx < conf.half_bound &&
          !om.IsAaBoxOccupied({x - off - lx, y - off - ly},
                              {x + off + hx + s, y + off + hy})) {
        hx += s;
      } else {
        f &= ~2;
      }
      if ((f & 4) && ly + hy < conf.half_bound &&
          !om.IsAaBoxOccupied({x - off - lx, y - off - ly - s},
                              {x + off + hx, y + off + hy})) {
        ly += s;
      } else {
        f &= ~4;
      }
      if ((f & 8) && ly + hy < conf.half_bound &&
          !om.IsAaBoxOccupied({x - off - lx, y - off - ly},
                              {x + off + hx, y + off + hy + s})) {
        hy += s;
      } else {
        f &= ~8;
      }
    }

    return AD4{lx, hx, ly, hy};
  };

  std::vector<AD4> bounds(n, {0, 0});
  for (auto i : {0, 1}) {
    bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = 0;
  }
  for (auto i : {n - 1, n - 2}) {
    if (isend)
      bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = 0.1;
    else
      bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = 0.0;
  }
  for (int i = 2; i < n - 2; ++i) bounds[i] = extend(pts[i][0], pts[i][1]);

  return bounds;
}

void AdjustStartAndEndPose(std::vector<AD3>& ref_pts) {
  /// start
  Vec2d ps{ref_pts[0][0], ref_pts[0][1]};
  double ls =
      std::hypot(ref_pts[1][0] - ref_pts[0][0], ref_pts[1][1] - ref_pts[0][1]);
  if (ls < 0.1) ls = 0.1;
  auto vs = Vec2d{ls, 0}.rotate(ref_pts[0][2]);
  auto start = ps + vs;
  ref_pts[1][0] = start.x();
  ref_pts[1][1] = start.y();

  /// end
  const auto n = ref_pts.size();
  Vec2d pe{ref_pts[n - 1][0], ref_pts[n - 1][1]};
  double le = std::hypot(ref_pts[n - 1][0] - ref_pts[n - 2][0],
                         ref_pts[n - 1][1] - ref_pts[n - 2][1]);
  if (le < 0.1) {
    le = 0.1;
  }
  auto ve = Vec2d{le, 0}.rotate(ref_pts[n - 1][2]);
  auto end = pe - ve;
  ref_pts[n - 2][0] = end.x();
  ref_pts[n - 2][1] = end.y();
}

bool IsCollisionFree(const OccupyMap& om, const std::vector<AD3>& pts,
                     std::vector<size_t>* collision_idx) {
  collision_idx->clear();
  for (size_t i = 0; i < pts.size(); ++i) {
    auto [x, y, theta] = pts[i];
    if (om.IsVehicleBoxOccupied({x, y, theta})) collision_idx->push_back(i);
  }

  return collision_idx->empty();
}

void AdjustBoundsRatio(const std::vector<AD3>& pts,
                       const std::vector<AD2>& curr,
                       const std::vector<size_t>& collision_idx,
                       const double step, std::vector<AD4>* bounds) {
  for (auto i : collision_idx) {
    auto [rx, ry, rt] = pts[i];
    auto [cx, cy] = curr[i];
    auto& [lx, hx, ly, hy] = (*bounds)[i];

    lx *= 0.5, hx *= 0.5, ly *= 0.5, hy *= 0.5;
    continue;

    Vec2d v{rx - cx, ry - cy};
    const double len = v.length();
    const double dx = v.x() / len * step, dy = v.y() / len * step;
    if (dx > 0) {
      lx -= std::min(lx, dx);
    } else {
      hx -= std::min(hx, -dx);
    }
    if (dy > 0) {
      ly -= std::min(ly, dy);
    } else {
      hy -= std::min(hy, -dy);
    }
  }
}

void AdjustBoundsGradient(const std::vector<AD3>& pts, const OccupyMap& om,
                          const std::vector<size_t>& collision_idx,
                          const double step, std::vector<AD4>* bounds) {
  auto& b = *bounds;
  for (auto i : collision_idx) {
    auto& [left, right, low, high] = b[i];
    auto [x, y, t] = pts[i];
    auto [dx, dy] = om.GetPointGradientDirection({x, y});
    if (dx) {
      left += -dx * step, right -= -dx * step;
      left = std::max(left, 0.);
      right = std::max(right, 0.);
    } else {
      left *= 0.8, right *= 0.8;
    }
    if (dy) {
      low += -dy * step, high -= -dy * step;
      low = std::max(low, 0.);
      high = std::max(high, 0.);
    } else {
      low *= 0.8, high *= 0.8;
    }
  }
}

void AdjustBoundsTwoPoints(const std::vector<AD3>& pts,
                           const std::vector<AD2>& curr,
                           const std::vector<size_t>& collision_idx,
                           const double step, std::vector<AD4>* bounds) {
  for (auto i : collision_idx) {
    auto [rx, ry, rt] = pts[i];
    auto [cx, cy] = curr[i];
    auto& [lx, hx, ly, hy] = (*bounds)[i];

    Vec2d v{rx - cx, ry - cy};
    const double len = v.length();
    const double dx = v.x() / len * step, dy = v.y() / len * step;
    if (dx > 0) {
      lx -= std::min(lx, dx);
    } else {
      hx -= std::min(hx, -dx);
    }
    if (dy > 0) {
      ly -= std::min(ly, dy);
    } else {
      hy -= std::min(hy, -dy);
    }
  }
}

std::vector<AD3> GeneratePathProfile(const std::vector<AD2>& xys,
                                     const double theta) {
  std::vector<AD3> ans{};
  for (size_t i = 0; i < xys.size() - 1; ++i) {
    ans.push_back(
        {xys[i][0], xys[i][1],
         std::atan2(xys[i + 1][1] - xys[i][1], xys[i + 1][0] - xys[i][0])});
  }
  ans.push_back({xys.back()[0], xys.back()[1], theta});

  return ans;
}

}  // namespace

std::vector<AD3> IterativeAncheringPathSmoother::Smooth(
    const OccupyMap& om, std::vector<AD3> ref_pts, const bool isend) {
  const auto pt_cnt = ref_pts.size();
  if (pt_cnt < 3) {
    LOG_INFO("input reference point size is less than 3");
    return {};
  }

  AdjustStartAndEndPose(ref_pts);

  std::vector<AD2> ref_xys(pt_cnt, {0, 0});
  for (size_t i = 0; i < pt_cnt; ++i) {
    ref_xys[i][0] = ref_pts[i][0];
    ref_xys[i][1] = ref_pts[i][1];
  }
  auto bounds = BuildBounds(conf_, ref_pts, om, isend);

  FreespaceSmoother fem_pos_smoother{{
      .weight_smooth = 1e10,
  }};

  bool is_collision_free{false};
  std::vector<size_t> collision_idx{};
  int step = 0;
  auto curr_xys = ref_xys;
  auto curr_pts = ref_pts;
  while (!is_collision_free) {
    if (++step >= conf_.max_iterative_loop) {
      LOG_INFO("IAPS failed, reach max iterative loop {}", step);
      break;
    }
    if (curr_xys = fem_pos_smoother.Smooth(ref_xys, bounds); curr_xys.empty()) {
      LOG_INFO("fem_pos smooth failed!");
      break;
    }

    curr_pts = GeneratePathProfile(curr_xys, ref_pts.back()[2]);

    is_collision_free = IsCollisionFree(om, curr_pts, &collision_idx);
    // AdjustBoundsTwoPoints(
    //     ref_pts, curr_xys, collision_idx, conf_.bound_move_step, &bounds);
    AdjustBoundsRatio(ref_pts, curr_xys, collision_idx, conf_.bound_move_step,
                      &bounds);
    // AdjustBoundsGradient(
    //      ref_pts, om, collision_idx, conf_.bound_move_step, &bounds);
  }
  LOG_INFO("smoother iteration step {}", step);

  return curr_pts;
}

}  // namespace planning
}  // namespace neodrive
