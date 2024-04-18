#include "region_decision_graph_search.h"

#include <algorithm>

#include "common/data_center/data_center.h"
#include "common/math/vec2d.h"
#include "common/vehicle_param.h"
#include "common/visualizer_event/visualizer_event.h"

namespace neodrive {
namespace planning {

namespace {

constexpr double kLateralJumpTol = 0.8;
constexpr double kNodeWidthRatio = 20.;
constexpr double kNodeOffsetRatio = 1.;
constexpr double kEdgeRatio = 100.;
constexpr double kSampleDis = 0.1;
constexpr double kLaneRoadDiff = 0.3;
constexpr double kLaneBorrowRatio = 2.0;
constexpr double kHistoryCostRatio = 3.0;
constexpr double kEdgeOffset = 20.;

const double kMinPassableLen = VehicleParam::Instance()->width() + 0.4;

std::vector<AABox2d> FiltSortedBoxes(const std::vector<AABox2d>& boxes,
                                     const double start_s, const double end_s,
                                     const double start_l, const double end_l) {
  std::vector<AABox2d> ans{};
  for (auto& b : boxes) {
    if (b.min_x() > end_s || b.max_x() < start_s || b.min_y() > end_l ||
        b.max_y() < start_l)
      continue;
    ans.push_back(b);
  }
  std::sort(ans.begin(), ans.end(),
            [](auto& a, auto& b) { return a.min_x() < b.min_x(); });
  return ans;
}

std::array<std::vector<double>, 2> GetSortedFrenetScales(
    const std::vector<AABox2d>& boxes, const std::vector<SLPoint>& upper_bound,
    const std::vector<SLPoint>& lower_bound, const double start_s,
    const double end_s, const double start_l, const double end_l) {
  std::vector<double> s_scales{start_s, end_s};
  for (auto& b : boxes) {
    if (b.min_x() > start_s && b.min_x() < end_s) s_scales.push_back(b.min_x());
    if (b.max_x() > start_s && b.max_x() < end_s) s_scales.push_back(b.max_x());
  }
  for (size_t i = 1; i < upper_bound.size(); ++i) {
    if (std::abs(upper_bound[i].l() - upper_bound[i - 1].l()) > kLateralJumpTol)
      s_scales.push_back(upper_bound[i].s());
  }
  for (size_t i = 1; i < lower_bound.size(); ++i) {
    if (std::abs(lower_bound[i].l() - lower_bound[i - 1].l()) > kLateralJumpTol)
      s_scales.push_back(lower_bound[i].s());
  }
  std::sort(s_scales.begin(), s_scales.end());
  decltype(s_scales) us{s_scales.front()};
  for (auto s : s_scales)
    if (std::abs(s - us.back()) > 1e-5) us.push_back(s);
  std::swap(s_scales, us);

  std::vector<double> l_scales{start_l, end_l};
  for (auto& b : boxes) {
    if (b.min_y() > start_l && b.min_y() < end_l) l_scales.push_back(b.min_y());
    if (b.max_y() > start_l && b.max_y() < end_l) l_scales.push_back(b.max_y());
  }
  std::sort(l_scales.begin(), l_scales.end());
  decltype(l_scales) ul{l_scales.front()};
  for (auto l : l_scales)
    if (std::abs(l - ul.back()) > 1e-5) ul.push_back(l);
  std::swap(l_scales, ul);

  return {s_scales, l_scales};
}

/// Binary search left bound (a[l] >= v)
int LbsIdx(const std::vector<double>& a, const double v) {
  int m = static_cast<int>(a.size());
  if (v < a[0]) return 0;
  if (v > a[m - 2]) return m - 2;
  int l = 0, r = m - 2;
  while (l < r) {
    int mid = l + (r - l) / 2;
    if (a[mid] < v) {
      l = mid + 1;
    } else {
      r = mid;
    }
  }

  return l;
}

struct RegionNode {
  int index{0};

  double s0{0};
  double upper_l0{0};
  double lower_l0{0};

  double s1{0};
  double upper_l1{0};
  double lower_l1{0};

  bool is_upper_obs{false};
  bool is_lower_obs{false};

  double cost{0};
  double width_cost{0.};
  double offset_cost{0.};
  double history_path_cost{0.};
  double lane_borrow_cost{0.};

  std::unordered_set<int> prevs{};
  std::unordered_map<int, double> next_costs{};
};

void VisOriginBound(ReferenceLinePtr ref_line,
                    const std::vector<SLPoint>& left_bound,
                    const std::vector<SLPoint>& right_bound) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event =
      vis::EventSender::Instance()->GetEvent("region_origin_bound_test");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto& p : left_bound) {
    Vec2d pt{};
    auto sphere = event->mutable_sphere()->Add();
    ref_line->GetPointInCartesianFrame({p.s(), p.l()}, &pt);
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
  for (auto& p : right_bound) {
    Vec2d pt{};
    auto sphere = event->mutable_sphere()->Add();
    ref_line->GetPointInCartesianFrame({p.s(), p.l()}, &pt);
    set_pt(sphere->mutable_center(), pt);
    sphere->set_radius(0.05);
  }
}

void VisBound(ReferenceLinePtr ref_line,
              const std::vector<PathRegion::Bound>& bounds) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("region_lane_bound_test");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  for (auto& b : bounds) {
    Vec2d pt{};
    {
      auto sphere = event->mutable_sphere()->Add();
      ref_line->GetPointInCartesianFrame({b.lower_point.s(), b.lower_point.l()},
                                         &pt);
      set_pt(sphere->mutable_center(), pt);
      sphere->set_radius(0.05);
    }
    {
      auto sphere = event->mutable_sphere()->Add();
      ref_line->GetPointInCartesianFrame({b.upper_point.s(), b.upper_point.l()},
                                         &pt);
      set_pt(sphere->mutable_center(), pt);
      sphere->set_radius(0.05);
    }
  }
}

std::vector<PathRegion::Bound> BuildBound(
    const std::vector<AABox2d>& boxes, const std::vector<RegionNode>& nodes,
    const std::vector<int>& path, const std::vector<SLPoint>& left_bound,
    const std::vector<SLPoint>& right_bound,
    const std::vector<SLPoint>& left_road_bound,
    const std::vector<SLPoint>& right_road_bound, const double start_s,
    const double end_s) {
  std::vector<PathRegion::Bound> ans{};
  if (path.empty()) return ans;

  // Get path obstacle info
  struct ObsInfo {
    int upper{-1};
    int lower{-1};
  };
  auto around = [](auto a, auto b) { return std::abs(a - b) < 1e-2; };
  std::unordered_map<int, ObsInfo> node_obs_info{};
  for (size_t i = 0, j = 0; i < path.size(); ++i) {
    auto& node = nodes[path[i]];
    while (j < boxes.size() && boxes[j].max_x() < node.s0) ++j;
    for (size_t k = j; k < boxes.size() && boxes[k].min_x() < node.s1; ++k) {
      if (around(boxes[k].min_y(), node.upper_l0)) {
        node_obs_info[node.index].upper = boxes[k].id();
      } else if (around(boxes[k].max_y(), node.lower_l0)) {
        node_obs_info[node.index].lower = boxes[k].id();
      }
    }
  }

  // Set bound
  auto type = [](auto id, auto lb, auto rb) {
    using Type = PathRegion::Bound::BoundType;
    if (id >= 0) return Type::OBS;
    return std::abs(lb - rb) < kLaneRoadDiff ? Type::ROAD : Type::LANE;
  };
  size_t i = 0, j = 0;
  for (double s = start_s; s < end_s; s += kSampleDis) {
    while (i < path.size() && nodes[path[i]].s1 < s) ++i;
    while (j < left_bound.size() && left_bound[j].s() < s) ++j;
    if (i >= path.size() || j >= left_bound.size()) break;
    ans.push_back(PathRegion::Bound{
        .upper_point{s, nodes[path[i]].upper_l0},
        .lower_point{s, nodes[path[i]].lower_l0},
        .upper_lane_bound_point{s, left_bound[j].l()},
        .lower_lane_bound_point{s, right_bound[j].l()},
        .upper_road_bound_point{s, left_road_bound[j].l()},
        .lower_road_bound_point{s, right_road_bound[j].l()},
        .upper_id = node_obs_info[nodes[path[i]].index].upper,
        .lower_id = node_obs_info[nodes[path[i]].index].lower,
        .upper_type = type(node_obs_info[nodes[path[i]].index].upper,
                           left_bound[j].l(), left_road_bound[j].l()),
        .lower_type = type(node_obs_info[nodes[path[i]].index].lower,
                           right_bound[j].l(), right_road_bound[j].l()),
    });
    if (ans.back().upper_id == -1) {
      ans.back().upper_point.set_l(left_bound[j].l());
    } else {
      ans.back().upper_point.set_l(
          std::min(ans.back().upper_point.l(), left_bound[j].l()));
    }
    if (ans.back().lower_id == -1) {
      ans.back().lower_point.set_l(right_bound[j].l());
    } else {
      ans.back().lower_point.set_l(
          std::max(ans.back().lower_point.l(), right_bound[j].l()));
    }
    if (auto& b = ans.back();
        b.upper_point.l() - b.lower_point.l() < kMinPassableLen) {
      while (!ans.empty() &&
             (ans.back().upper_id != -1 || ans.back().lower_id != -1)) {
        ans.pop_back();
      }
      break;
    }
  }
  return ans;
}

/// 2d grid, 0 for free, >0 for obstacle
std::vector<std::vector<int>> GetOccupyGrid(const std::vector<double>& s_scales,
                                            const std::vector<double>& l_scales,
                                            const std::vector<AABox2d>& boxes) {
  auto insert = [](auto& p, int r1, int c1, int r2, int c2, int t) {
    p[r1][c1] += t;
    p[r2 + 1][c1] -= t;
    p[r1][c2 + 1] -= t;
    p[r2 + 1][c2 + 1] += t;
  };

  int m = s_scales.size(), n = l_scales.size();
  std::vector p(m + 1, std::vector<int>(n + 1, 0));  // pre diff matrix

  for (auto& b : boxes) {
    auto r1 = LbsIdx(s_scales, b.min_x() - 1e-2);
    auto r2 = LbsIdx(s_scales, b.max_x() - 1e-2);
    if (std::abs(b.max_x() - s_scales[r2]) < 1e-2) --r2;
    auto c1 = LbsIdx(l_scales, b.min_y() - 1e-2);
    auto c2 = LbsIdx(l_scales, b.max_y() - 1e-2);
    if (std::abs(b.max_y() - l_scales[c2]) < 1e-2) --c2;
    insert(p, r1 + 1, c1 + 1, r2 + 1, c2 + 1, 1);
  }

  for (int i = 1; i <= m; ++i)
    for (int j = 1; j <= n; ++j) {
      p[i][j] += p[i - 1][j] + p[i][j - 1] - p[i - 1][j - 1];
    }

  // m * n scales construct (m - 1) * (n - 1) grids
  // grid[i][j] for the grid which lb point is (s_scales[i], l_scales[j])
  std::vector ans(m - 1, std::vector<int>(n - 1));
  for (int i = 1; i < m; ++i)
    for (int j = 1; j < n; ++j) ans[i - 1][j - 1] = p[i][j];
  return ans;
}

std::vector<RegionNode> GenerateRegionGraph(
    const std::vector<double>& s_scales, const std::vector<double>& l_scales,
    std::vector<std::vector<int>>& occupy_grid, const double init_l,
    const std::vector<SLPoint>& upper_bound,
    const std::vector<SLPoint>& lower_bound) {
  std::vector<RegionNode> ans{};
  if (s_scales.size() < 2 || l_scales.size() < 2) return ans;
  auto& ss = s_scales;
  auto& ls = l_scales;
  int m = ss.size(), n = ls.size();

  /// s -> upper and lower l bound
  std::vector<std::array<double, 2>> s_u_l_bound(m, {0, 0});
  for (size_t i = 0, bi = 0; i < m; ++i) {
    while (bi < upper_bound.size() - 1 && upper_bound[bi].s() < ss[i]) ++bi;
    s_u_l_bound[i] = {upper_bound[bi].l(), lower_bound[bi].l()};
  }
  auto s_u_l_end_bound = s_u_l_bound;
  for (size_t i = 1, bi = 0; i < m; ++i) {
    while (bi < upper_bound.size() - 1 && upper_bound[bi].s() < ss[i]) {
      if (auto len = upper_bound[bi].l() - lower_bound[bi].l();
          len < kMinPassableLen) {
        auto [ul, ll] = s_u_l_end_bound[i];
        ul = std::min(ul, upper_bound[bi].l());
        ll = std::max(ll, lower_bound[bi].l());
      }
      ++bi;
    }
  }

  auto GenNode = [&](int idx, int si, int li0, int li1) {
    auto& min_bound = s_u_l_end_bound;
    return RegionNode{
        .index = idx,
        .s0 = ss[si],
        .upper_l0 = std::clamp(ls[li1], s_u_l_bound[si][1], s_u_l_bound[si][0]),
        .lower_l0 = std::clamp(ls[li0], s_u_l_bound[si][1], s_u_l_bound[si][0]),

        .s1 = ss[si + 1],
        .upper_l1 =
            std::clamp(ls[li1], min_bound[si + 1][1], min_bound[si + 1][0]),
        .lower_l1 =
            std::clamp(ls[li0], min_bound[si + 1][1], min_bound[si + 1][0]),

        .is_upper_obs = li1 < occupy_grid[si].size() ? occupy_grid[si][li1] : 0,
        .is_lower_obs = li0 && occupy_grid[si][li0 - 1],
    };
  };

  /// Generate node 0
  int idx0 = LbsIdx(l_scales, init_l);
  if (idx0 && l_scales[idx0] > init_l) --idx0;
  int li0 = idx0, ri0 = idx0;
  while (li0 && occupy_grid[0][li0 - 1] == 0) --li0;
  while (ri0 < n - 1 && occupy_grid[0][++ri0] == 0) continue;
  ans.push_back(GenNode(0, 0, li0, ri0));

  /// Generate other layer nodes
  for (int si = 1; si < m - 1; ++si) {
    const double mins = ss[si], maxs = ss[si + 1];
    for (int l = 0; l < n - 1; ++l) {
      if (occupy_grid[si][l]) continue;
      int r = l + 1;
      while (r < n - 1 && !occupy_grid[si][r]) ++r;
      if (!(ls[l] > s_u_l_bound[si][0] || ls[r] < s_u_l_bound[si][1])) {
        ans.push_back(GenNode(static_cast<int>(ans.size()), si, l, r));
      }
      l = r;
    }
  }
  ans.push_back({static_cast<int>(ans.size()), ss.back(), s_u_l_bound.back()[0],
                 s_u_l_bound.back()[1], ss.back() + 0.1, s_u_l_bound.back()[0],
                 s_u_l_bound.back()[1]});

  /// Generate edges
  std::vector<int> layer_idxes{0};  // Start index of each layer
  for (size_t i = 1; i < ans.size(); ++i) {
    if (std::abs(ans[i].s0 - ans[i - 1].s0) < 1e-4) continue;
    layer_idxes.push_back(i);
  }
  layer_idxes.push_back(ans.size());
  for (int i = 1; i < static_cast<int>(layer_idxes.size()) - 1; ++i) {
    int l = layer_idxes[i - 1], r = l;  // sliding window
    for (int j = layer_idxes[i]; j < layer_idxes[i + 1]; ++j) {
      while (r < layer_idxes[i] && ans[r].lower_l1 < ans[j].upper_l0) ++r;
      while (l < r && ans[l].upper_l1 < ans[j].lower_l0) ++l;

      for (int k = l; k < r; ++k) ans[k].next_costs.insert({j, 0.});
      for (int k = l; k < r; ++k) ans[j].prevs.insert(k);
    }
  }

  return ans;
}

void VisAllNodes(ReferenceLinePtr ref_line,
                 const std::vector<RegionNode>& nodes,
                 const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };

  Vec2d pt{};
  for (auto& n : nodes) {
    auto polygon = event->mutable_polygon()->Add();
    ref_line->GetPointInCartesianFrame({n.s0 + 1e-2, n.lower_l0 + 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s1 - 1e-2, n.lower_l1 + 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s1 - 1e-2, n.upper_l1 - 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s0 + 1e-2, n.upper_l0 - 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
  }
}

std::vector<int> GetPathIndex(const std::vector<sim_planner::Vehicle>& sim_traj,
                              const std::vector<RegionNode>& region_nodes) {
  std::vector<int> path;
  for (const auto& pt : sim_traj) {
    sim_planner::SimMapPoint sm_pt;
    if (!sim_planner::SimMap::Instance()->getNearestPoint(
            pt.state().vec_position, &sm_pt)) {
      LOG_ERROR("get nearest point failed!");
    }
    for (const auto& node : region_nodes) {
      if (sm_pt.s > node.s1) continue;
      if (sm_pt.l >= std::min(node.lower_l0, node.lower_l1) &&
          sm_pt.l <= std::max(node.upper_l0, node.upper_l1)) {
        path.push_back(node.index);
        break;
      }
    }
  }
  std::sort(path.begin(), path.end());
  path.erase(std::unique(path.begin(), path.end()), path.end());
  return path;
}

}  // namespace

std::vector<PathRegion::Bound> RegionDecisionGraphSearch::GenerateBoundInfo(
    const std::vector<AABox2d>& obstacles,
    const std::vector<SLPoint>& left_bound,
    const std::vector<SLPoint>& right_bound,
    const std::vector<SLPoint>& left_road_bound,
    const std::vector<SLPoint>& right_road_bound, const double init_l,
    const double init_s, ReferenceLinePtr ref_line,
    const std::vector<sim_planner::Vehicle>& sim_traj) const {
  if (left_bound.empty() || left_bound.size() != right_bound.size() ||
      left_bound.size() != left_road_bound.size() ||
      left_bound.size() != right_road_bound.size())
    return {};

  VisOriginBound(ref_line, left_bound, right_bound);

  const double start_s = init_s, end_s = left_bound.back().s();
  const double start_l =
      std::min_element(right_bound.begin(), right_bound.end(),
                       [](auto& a, auto& b) { return a.l() < b.l(); })
          ->l();
  const double end_l =
      std::max_element(left_bound.begin(), left_bound.end(),
                       [](auto& a, auto& b) { return a.l() < b.l(); })
          ->l();
  auto boxes = FiltSortedBoxes(obstacles, start_s, end_s, start_l, end_l);

  auto [s_scales, l_scales] = GetSortedFrenetScales(
      boxes, left_bound, right_bound, start_s, end_s, start_l, end_l);

  /// Fill grids with obstacles
  auto occupy_grid = GetOccupyGrid(s_scales, l_scales, boxes);

  /// Generate graph
  auto region_nodes = GenerateRegionGraph(s_scales, l_scales, occupy_grid,
                                          init_l, left_bound, right_bound);
  VisAllNodes(ref_line, region_nodes, "region_graph");

  std::vector<int> path = GetPathIndex(sim_traj, region_nodes);

  /// Output
  auto ans = BuildBound(boxes, region_nodes, path, left_bound, right_bound,
                        left_road_bound, right_road_bound, start_s, end_s);
  VisBound(ref_line, ans);

  return ans;
}

}  // namespace planning
}  // namespace neodrive
