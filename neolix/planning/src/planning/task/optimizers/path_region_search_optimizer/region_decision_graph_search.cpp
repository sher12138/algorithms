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
constexpr double kNodeWidthRatio = 30.;
constexpr double kNodeOffsetRatio = 0.5;
constexpr double kEdgeRatio = 100.;
constexpr double kSampleDis = 0.1;
constexpr double kLaneRoadDiff = 0.3;
constexpr double kLaneBorrowRatio = 2.0;
constexpr double kHistoryCostRatio = 3.0;
constexpr double kEdgeOffset = 20.;

const double kMinPassableLen = VehicleParam::Instance()->width() + 0.2;

std::vector<AABox2d> FiltSortedBoxes(const std::vector<AABox2d>& boxes,
                                     const std::vector<AABox2d>& virtual_boxes,
                                     const double start_s, const double end_s,
                                     const double start_l, const double end_l) {
  auto& task_info = DataCenter::Instance()->task_info_list().front();
  std::vector<AABox2d> ans{};
  for (auto& box : virtual_boxes) {
    LOG_INFO(
        "virtual_boxes min_x: {:.4f}, max_x: {:.4f}, min_y: {:.4f}, max_y: "
        "{:.4f}, width: {:.4f}, length: {:.4f}",
        box.min_x(), box.max_x(), box.min_y(), box.max_y(), box.width(),
        box.length());
    if (box.min_x() > end_s || box.max_x() < start_s || box.min_y() > end_l ||
        box.max_y() < start_l) {
      LOG_INFO("box out bound.");
      continue;
    } else {
      ans.push_back(box);
      LOG_INFO("box in bound.");
    }
  }

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

std::vector<RegionNode> GenerateDecisionGraph(
    const std::vector<RegionNode>& region_nodes, const double init_l) {
  auto d_nodes = region_nodes;

  const double half_width = kMinPassableLen / 2.;
  d_nodes[0].upper_l0 = init_l + half_width;
  d_nodes[0].lower_l0 = init_l - half_width;
  d_nodes[0].upper_l1 = init_l + half_width;
  d_nodes[0].lower_l1 = init_l - half_width;

  for (int i = 1, m = static_cast<int>(region_nodes.size() - 1); i < m; ++i) {
    if (d_nodes[i].is_upper_obs || d_nodes[i].is_lower_obs) continue;
    for (auto p : d_nodes[i].prevs) {
      d_nodes[p].next_costs.erase(i);
      for (auto [n, c] : d_nodes[i].next_costs) {
        d_nodes[p].next_costs[n];
        d_nodes[n].prevs.insert(p);
      }
    }
    for (auto [n, c] : d_nodes[i].next_costs) d_nodes[n].prevs.erase(i);
    d_nodes[i].prevs.clear();
    d_nodes[i].next_costs.clear();
  }

  return d_nodes;
}

void CalcNodeCost(std::vector<RegionNode>& nodes,
                  const PathData* last_path_data,
                  const LaneBorrowContext::BorrowSide side,
                  const double& init_l) {
  /// width cost
  for (auto& node : nodes) {
    if (auto w = std::min(node.upper_l0 - node.lower_l0,
                          node.upper_l1 - node.lower_l1);
        w < kMinPassableLen) {
      node.width_cost = 1e5;
    } else {
      node.width_cost = kNodeWidthRatio / w;
    }
    node.cost = node.width_cost;
  }

  /// offset cost
  auto len = [](auto l0, auto r0, auto l1, auto r1) {
    if (l1 < l0 && r1 > r0) return r0 - l0;
    if (l0 > r1 || r0 < l1) return 1 / (3 * (r0 - l0));
    auto ans = std::clamp(r0 > l1 && r0 < r1 ? r0 - l1 : r1 - l0,
                          1 / (3 * (r0 - l0)), r0 - l0);
    return ans;
  };
  for (auto& node : nodes) {
    node.offset_cost =
        kNodeOffsetRatio /
        len(nodes[0].lower_l1, nodes[0].upper_l1, node.lower_l0, node.upper_l0);
    node.cost += node.offset_cost;
  }

  /// history path cost
  for (auto& node : nodes) {
    SLPoint center{
        (node.s0 + node.s1) / 2,
        (node.upper_l0 + node.upper_l1 + node.lower_l0 + node.lower_l1) / 4};
    FrenetFramePoint proj_point{};
    if (last_path_data &&
        last_path_data->frenet_path().interpolate(center.s(), proj_point)) {
      node.history_path_cost =
          kHistoryCostRatio * std::abs(center.l() - proj_point.l());
    }
    node.cost += node.history_path_cost;
  }

  /// lane borrow cost
  // if (side == LaneBorrowContext::BorrowSide::None) return;
  const double ref_l = side == LaneBorrowContext::BorrowSide::Left ? 10 : -10;
  for (auto& node : nodes) {
    auto l =
        (node.upper_l0 + node.lower_l0 + node.upper_l1 + node.lower_l1) / 4;
    // node.cost += std::abs(l - ref_l) * kLaneBorrowRatio;
    node.lane_borrow_cost = std::abs(l - init_l) * kLaneBorrowRatio;
    node.cost += node.lane_borrow_cost;
  }
}

void CalcEdgeCost(std::vector<RegionNode>& nodes) {
  auto overlap = [](auto l1, auto r1, auto l2, auto r2) {
    if (r1 < l2) return r1 - l2;
    if (r2 < l1) return r2 - l1;
    return r1 > l2 && r1 < r2 ? r1 - l2 : r2 - l1;
  };
  for (auto& node : nodes) {
    for (auto& [next, cost] : node.next_costs) {
      auto& child = nodes[next];
      if (auto l = overlap(node.lower_l1, node.upper_l1, child.lower_l0,
                           child.upper_l0);
          l < kMinPassableLen) {
        cost = kEdgeRatio / (l + kEdgeOffset);
      }
    }
  }
}

/// Ksp
std::vector<std::vector<int>> KspPathSearch(std::vector<RegionNode>& nodes,
                                            const int start, const int end,
                                            int k) {
  if (nodes.empty()) return {};
  using PDI = std::pair<double, int>;

  // dijkstra
  auto dijkstra = [](auto& nodes, auto from) {
    std::vector<double> cost(nodes.size(), 1e10);
    std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq{
        std::greater<PDI>()};

    cost[from] = 1e-5;
    pq.push({0., from});
    while (!pq.empty()) {
      auto [g, node] = pq.top();
      pq.pop();

      if (g > cost[node]) continue;
      for (auto prev : nodes[node].prevs) {
        if (cost[node] + nodes[prev].next_costs[node] + nodes[node].cost <
            cost[prev]) {
          cost[prev] =
              cost[node] + nodes[prev].next_costs[node] + nodes[node].cost;
          pq.push({cost[prev], prev});
        }
      }
    }
    return cost;
  };

  auto cost_to_end = dijkstra(nodes, nodes.size() - 1);

  auto path = [](const auto& parent, auto from, auto to) {
    std::vector<int> ans{};
    while (to != from && to != -1) {
      ans.push_back(to);
      to = parent[to];
    }
    ans.push_back(from);
    std::reverse(ans.begin(), ans.end());

    return ans;
  };

  // A*
  std::vector<std::vector<int>> ans{};
  std::vector<double> cost(nodes.size(), 1e15);
  std::vector<int> parent(nodes.size(), -1);
  std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq{
      std::greater<PDI>()};

  cost[start] = 0.;
  pq.push({0., start});
  while (!pq.empty()) {
    auto [f, node] = pq.top();
    pq.pop();

    if (node == end) {
      if (ans.size() && f > 1e5) break;
      ans.push_back(path(parent, start, end));
      if (--k == 0) break;
    }

    for (auto [next, edge_cost] : nodes[node].next_costs) {
      cost[next] = cost[node] + edge_cost + nodes[next].cost;
      pq.push({cost[next] + cost_to_end[next], next});
      parent[next] = node;
    }

    if (pq.empty()) {
      ans.push_back(path(parent, start, node));
      LOG_INFO("path start[{}] to node[{}]", start, node);
      break;
    }
  }

  return ans;
}

std::vector<int> GetRegionPath(std::vector<RegionNode>& nodes,
                               std::vector<int> path) {
  std::vector<int> ans{};
  if (path.empty() || nodes.empty()) return ans;

  std::reverse(path.begin(), path.end());
  ans.push_back(path.back());
  path.pop_back();
  while (path.size()) {
    if (nodes[ans.back()].next_costs.count(path.back())) {
      ans.push_back(path.back());
      path.pop_back();
    } else if (nodes[ans.back()].next_costs.size() == 1) {
      ans.push_back(nodes[ans.back()].next_costs.begin()->first);
    } else {
      break;
    }
  }

  return ans;
}

void VisPathNodes(ReferenceLinePtr ref_line,
                  const std::vector<RegionNode>& nodes,
                  const std::vector<int>& path, const std::string& name) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent(name);
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pt = [](auto ans, auto& p) {
    ans->set_x(p.x());
    ans->set_y(p.y());
    ans->set_z(0);
  };
  auto center = [](auto& n) {
    return SLPoint{(n.s0 + n.s1) / 2,
                   (n.upper_l0 + n.upper_l1 + n.lower_l0 + n.lower_l1) / 4};
  };

  Vec2d pt{};
  for (auto& i : path) {
    auto& n = nodes[i];
    auto polygon = event->mutable_polygon()->Add();
    ref_line->GetPointInCartesianFrame({n.s0 + 1e-2, n.lower_l0 + 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s1 - 1e-2, n.lower_l1 + 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s1 - 1e-2, n.upper_l1 - 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);
    ref_line->GetPointInCartesianFrame({n.s0 + 1e-2, n.upper_l0 - 1e-2}, &pt);
    set_pt(polygon->add_point(), pt);

    auto text = event->mutable_text()->Add();
    ref_line->GetPointInCartesianFrame(center(n), &pt);
    set_pt(text->mutable_position(), pt);
    text->set_text("node: " + std::to_string(n.index));
  }
}

void VisOriginBound(ReferenceLinePtr ref_line,
                    const std::vector<SLPoint>& left_bound,
                    const std::vector<SLPoint>& right_bound) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("region_origin_bound");
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

  auto event = vis::EventSender::Instance()->GetEvent("region_lane_bound");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto event_road = vis::EventSender::Instance()->GetEvent("road_bound");
  event_road->set_type(visualizer::Event::k3D);
  event_road->add_attribute(visualizer::Event::kOdom);

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
    {
      auto sphere = event_road->mutable_sphere()->Add();
      ref_line->GetPointInCartesianFrame(
          {b.lower_road_bound_point.s(), b.lower_road_bound_point.l()}, &pt);
      set_pt(sphere->mutable_center(), pt);
      sphere->set_radius(0.05);
    }
    {
      auto sphere = event_road->mutable_sphere()->Add();
      ref_line->GetPointInCartesianFrame(
          {b.upper_road_bound_point.s(), b.upper_road_bound_point.l()}, &pt);
      set_pt(sphere->mutable_center(), pt);
      sphere->set_radius(0.05);
    }
  }
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

  auto center = [](auto& n) {
    return SLPoint{(n.s0 + n.s1) / 2,
                   (n.upper_l0 + n.upper_l1 + n.lower_l0 + n.lower_l1) / 4};
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

    auto text = event->mutable_text()->Add();
    ref_line->GetPointInCartesianFrame(center(n), &pt);
    set_pt(text->mutable_position(), pt);
    text->set_text(
        "node: " + std::to_string(n.index) +
        "\ncost: " + std::to_string(n.cost) +
        "\nwidth_cost: " + std::to_string(n.width_cost) +
        "\noffset_cost: " + std::to_string(n.offset_cost) +
        "\nhistory_path_cost: " + std::to_string(n.history_path_cost) +
        "\nref_cost: " + std::to_string(n.lane_borrow_cost));
  }

  for (auto& n : nodes) {
    auto c1 = center(n);
    ref_line->GetPointInCartesianFrame(c1, &pt);
    auto curr = pt;

    for (auto [child, edge_cost] : n.next_costs) {
      auto& next = nodes[child];
      auto polyline = event->mutable_polyline()->Add();
      set_pt(polyline->add_point(), curr);
      auto sphere = event->mutable_sphere()->Add();
      set_pt(sphere->mutable_center(), curr);
      sphere->set_radius(0.05);
      auto c2 = center(next);
      ref_line->GetPointInCartesianFrame(c2, &pt);
      set_pt(polyline->add_point(), pt);
      sphere = event->mutable_sphere()->Add();
      set_pt(sphere->mutable_center(), pt);
      sphere->set_radius(0.05);

      auto text = event->mutable_text()->Add();
      ref_line->GetPointInCartesianFrame(
          {(c1.s() + c2.s()) / 2, (c1.l() + c2.l() / 2)}, &pt);
      set_pt(text->mutable_position(), pt);
      text->set_text("edge: " + std::to_string(n.index) + " -> " +
                     std::to_string(next.index) +
                     "\ncost: " + std::to_string(edge_cost));
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
    int upper_obs_id{-9999};   // left obs id
    int lower_obs_id{-9999};   // right obs id
    int upper_obs_type{-1};  // corresponding Obstacle::ObstacleType
    int lower_obs_type{-1};  // corresponding Obstacle::ObstacleType
  };
  auto around = [](auto a, auto b) { return std::abs(a - b) < 1e-2; };
  std::unordered_map<int, ObsInfo> node_obs_info{};
  for (size_t i = 0, j = 0; i < path.size(); ++i) {
    auto& node = nodes[path[i]];
    while (j < boxes.size() && boxes[j].max_x() < node.s0) ++j;
    for (size_t k = j; k < boxes.size() && boxes[k].min_x() < node.s1; ++k) {
      if (around(boxes[k].min_y(), node.upper_l0)) {
        node_obs_info[node.index].upper_obs_id = boxes[k].id();
        node_obs_info[node.index].upper_obs_type = boxes[k].obs_type();
      } else if (around(boxes[k].max_y(), node.lower_l0)) {
        node_obs_info[node.index].lower_obs_id = boxes[k].id();
        node_obs_info[node.index].lower_obs_type = boxes[k].obs_type();
      }
    }
  }

  // Set bound
  auto get_region_bound_type = [](auto id, auto lb, auto rb) {
    using Type = PathRegion::Bound::BoundType;
    if (id < 0)
      return std::abs(lb - rb) < kLaneRoadDiff ? Type::ROAD : Type::LANE;
    else if (id == 3) {
      return Type::VIR;
    } else {
      return Type::OBS;
    }
  };
  auto interpolate = [](auto& bound, auto& j, auto s) {
    return bound[j - 1].l() + (bound[j].l() - bound[j - 1].l()) /
                                  (bound[j].s() - bound[j - 1].s()) *
                                  (s - bound[j - 1].s());
  };
  size_t i = 0, j = 1;
  for (double s = start_s; s < end_s; s += kSampleDis) {
    while (i < path.size() && nodes[path[i]].s1 < s) ++i;
    while (j < left_bound.size() && left_bound[j].s() < s) ++j;
    if (i >= path.size() || j >= left_bound.size()) break;
    ans.push_back(PathRegion::Bound{
        .upper_point{s, nodes[path[i]].upper_l0},
        .lower_point{s, nodes[path[i]].lower_l0},
        .upper_lane_bound_point{s, interpolate(left_bound, j, s)},
        .lower_lane_bound_point{s, interpolate(right_bound, j, s)},
        .upper_road_bound_point{s, interpolate(left_road_bound, j, s)},
        .lower_road_bound_point{s, interpolate(right_road_bound, j, s)},
        .upper_id = node_obs_info[nodes[path[i]].index].upper_obs_id,
        .lower_id = node_obs_info[nodes[path[i]].index].lower_obs_id,
        .upper_obs_type = node_obs_info[nodes[path[i]].index].upper_obs_type,
        .lower_obs_type = node_obs_info[nodes[path[i]].index].lower_obs_type,
        .upper_type = get_region_bound_type(
            node_obs_info[nodes[path[i]].index].upper_obs_id, left_bound[j].l(),
            left_road_bound[j].l()),
        .lower_type = get_region_bound_type(
            node_obs_info[nodes[path[i]].index].lower_obs_id,
            right_bound[j].l(), right_road_bound[j].l()),
    });
    if (ans.back().upper_id == -1) {
      ans.back().upper_point.set_l(interpolate(left_bound, j, s));
    } else {
      ans.back().upper_point.set_l(
          std::min(ans.back().upper_point.l(), interpolate(left_bound, j, s)));
    }
    if (ans.back().lower_id == -1) {
      ans.back().lower_point.set_l(interpolate(right_bound, j, s));
    } else {
      ans.back().lower_point.set_l(
          std::max(ans.back().lower_point.l(), interpolate(right_bound, j, s)));
    }
    if (ans.back().upper_point.l() - ans.back().lower_point.l() <
        kMinPassableLen) {
      int vehicle_obs_type = static_cast<int>(Obstacle::ObstacleType::VEHICLE);
      while (!ans.empty() && (ans.back().upper_obs_type == vehicle_obs_type ||
                              ans.back().lower_obs_type == vehicle_obs_type)) {
        ans.pop_back();
      }
      break;
    }
  }
  return ans;
}

std::vector<std::vector<int>> FiltValidRegionPath(
    const std::vector<RegionNode>& nodes,
    const std::vector<std::vector<int>>& region_ksp) {
  std::vector<std::vector<int>> ans{};
  std::vector<std::vector<int>> secondary_select_ans{};
  std::map<double, std::vector<int>> cut_off_ans{};
  if (region_ksp.empty()) return ans;
  const double veh_len = VehicleParam::Instance()->length();
  auto low = [&nodes](auto a) {
    return std::min(nodes[a].upper_l0, nodes[a].upper_l1);
  };
  auto up = [&nodes](auto a) {
    return std::max(nodes[a].lower_l0, nodes[a].lower_l1);
  };

  for (const auto& path : region_ksp) {
    bool is_valid = true;
    size_t window_begin = 0;
    for (size_t i = 0, j = 0; i < path.size(); ++i) {
      while (j < i && nodes[path[i]].s1 - nodes[path[j]].s1 > veh_len) ++j;
      /// Sliding-window property can be optimized with ordered container
      double min_width = 1e5;
      size_t start_idx = 0;
      if (i == j) {
        start_idx =
            (j > window_begin + 1 &&
             nodes[path[j - 1]].s1 - nodes[path[j - 1]].s0 < 0.5 * veh_len)
                ? j - 2
                : j;
      } else {
        start_idx = (j == i - 1 && j > window_begin) ? j - 1 : j;
      }
      if (start_idx == i) {
        min_width = std::min(nodes[path[j]].upper_l0 - nodes[path[j]].lower_l0,
                             nodes[path[i]].upper_l1 - nodes[path[i]].lower_l1);
      } else {
        double up_min =
            std::min(nodes[path[start_idx]].upper_l1, nodes[path[i]].upper_l0);
        double low_max =
            std::max(nodes[path[start_idx]].lower_l1, nodes[path[i]].lower_l0);
        for (size_t k = start_idx + 1; k < i; ++k) {
          up_min = std::min(up_min, low(path[k]));
          low_max = std::max(low_max, up(path[k]));
        }
        min_width = std::min(min_width, up_min - low_max);
      }
      if (min_width < kMinPassableLen) {
        LOG_INFO("reject!, min width {:.4f} from {} to {}", min_width,
                 nodes[path[j]].index, nodes[path[i]].index);
        if (min_width < VehicleParam::Instance()->width()) {
          std::vector<int> cut_path = path;
          cut_path.erase(cut_path.begin() + i + 1, cut_path.end());
          if (cut_path.size() > 1) {
            cut_off_ans[min_width] = cut_path;
          }
        } else {
          secondary_select_ans.push_back(path);
        }
        is_valid = false;
        break;
      }
      window_begin = j;
    }
    if (is_valid) ans.push_back(path);
  }
  if (ans.empty()) {
    if (secondary_select_ans.empty() && cut_off_ans.empty()) {
      ans.push_back(region_ksp.front());
    } else if (secondary_select_ans.empty()) {
      auto map_end = cut_off_ans.end();
      map_end--;
      ans.push_back(map_end->second);
    } else {
      ans.insert(ans.end(), secondary_select_ans.begin(),
                 secondary_select_ans.end());
    }
  }
  return ans;
}

}  // namespace

std::vector<PathRegion::Bound> RegionDecisionGraphSearch::GenerateBoundInfo(
    const std::vector<AABox2d>& obstacles,
    const std::vector<AABox2d>& virtual_boxes,
    const std::vector<SLPoint>& left_bound,
    const std::vector<SLPoint>& right_bound,
    const std::vector<SLPoint>& left_road_bound,
    const std::vector<SLPoint>& right_road_bound,
    const PathData* last_path_data, const double init_l, const double init_s,
    const LaneBorrowContext::BorrowSide lane_side, ReferenceLinePtr ref_line,
    const double ref_l) const {
  LOG_INFO("GenerateBoundInfo");
  if (left_bound.empty() || left_bound.size() != right_bound.size() ||
      left_bound.size() != left_road_bound.size() ||
      left_bound.size() != right_road_bound.size())
    return {};

  VisOriginBound(ref_line, left_bound, right_bound);
  /// Generate configure space
  const double start_s = init_s, end_s = left_bound.back().s();
  const double start_l =
      std::min_element(right_bound.begin(), right_bound.end(),
                       [](auto& a, auto& b) { return a.l() < b.l(); })
          ->l();
  const double end_l =
      std::max_element(left_bound.begin(), left_bound.end(),
                       [](auto& a, auto& b) { return a.l() < b.l(); })
          ->l();
  auto boxes =
      FiltSortedBoxes(obstacles, virtual_boxes, start_s, end_s, start_l, end_l);

  /// Generate grids
  auto [s_scales, l_scales] = GetSortedFrenetScales(
      boxes, left_bound, right_bound, start_s, end_s, start_l, end_l);

  /// Fill grids with obstacles
  auto occupy_grid = GetOccupyGrid(s_scales, l_scales, boxes);

  /// Generate graph
  auto region_nodes = GenerateRegionGraph(s_scales, l_scales, occupy_grid,
                                          init_l, left_bound, right_bound);
  VisAllNodes(ref_line, region_nodes, "region_graph");

  auto decision_nodes = GenerateDecisionGraph(region_nodes, init_l);
  CalcNodeCost(decision_nodes, last_path_data, lane_side, ref_l);
  CalcEdgeCost(decision_nodes);
  VisAllNodes(ref_line, decision_nodes, "decision_graph");

  /// ksp path search
  auto ksp = KspPathSearch(decision_nodes, 0, decision_nodes.size() - 1, 16);
  std::vector<std::vector<int>> region_ksp{};
  for (auto& sp : ksp) region_ksp.push_back(GetRegionPath(region_nodes, sp));
  region_ksp = FiltValidRegionPath(region_nodes, region_ksp);
  if (region_ksp.empty()) {
    LOG_ERROR("No valid path");
    return {};
  }
  for (size_t i = 0; i < region_ksp.size(); ++i)
    VisPathNodes(ref_line, region_nodes, region_ksp[i],
                 "path top" + std::to_string(i));

  /// Output
  region_ksp.front().pop_back();
  auto ans = BuildBound(boxes, region_nodes, region_ksp.front(), left_bound,
                        right_bound, left_road_bound, right_road_bound, start_s,
                        end_s);
  VisBound(ref_line, ans);

  return ans;
}

}  // namespace planning
}  // namespace neodrive
