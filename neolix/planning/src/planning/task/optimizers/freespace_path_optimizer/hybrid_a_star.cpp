#include "src/planning/task/optimizers/freespace_path_optimizer/hybrid_a_star.h"

#include <queue>
#include <tuple>
#include <unordered_set>

#include "neolix_log.h"
#include "src/common/angles/angles.h"
#include "src/planning/task/optimizers/freespace_path_optimizer/reeds_shepp_path.h"
#include "time/time.h"

namespace neodrive {
namespace planning {

namespace {

using AD2 = HybridAStar::AD2;
using AD3 = HybridAStar::AD3;
using AD4 = HybridAStar::AD4;
using PDI = std::pair<double, int>;
using AI2 = std::array<int, 2>;
using Node = HybridAStar::Node;
using Config = HybridAStar::Config;
using PiecePath = HybridAStar::PiecePath;

using namespace neodrive::common::angles;

double GoCost(const Node& from, const Node& to, const bool left_pass_obs) {
  double change_dir_cost =
      ((from.is_forward && !to.is_forward && from.idx != 0) ? 100.0 : 0.0);
  double dir_cost = (to.is_forward ? 0.0 : 100.0);
  double travel_cost = (to.is_forward ? 50.0 : 30.0);
  double steer_cost = std::abs(to.steer) * 20.0;
  // optimize the backward heading
  if (!to.is_forward && ((left_pass_obs && to.steer < -0.05) ||
                         (!left_pass_obs && to.steer > 0.05))) {
    steer_cost *= 0.0;
  }

  return change_dir_cost + steer_cost + travel_cost;
  // return change_dir_cost + dir_cost + steer_cost + 50.0;
}

std::vector<std::vector<double>> GenerateDijkstraShortestPath2d(
    const AD2& base, const double step, const double width, const AI2& size,
    const OccupyMap& om, const AD2& from) {
  std::vector grid(size[0], std::vector<double>(size[1], 1e7));
  const AI2 start{static_cast<int>((from[0] - base[0]) / step),
                  static_cast<int>((from[1] - base[1]) / step)};
  const double off = width + 0.2;

  int go[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                  {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  using TDII = std::tuple<double, int, int>;
  std::priority_queue<TDII, std::vector<TDII>, std::greater<TDII>> pq{};
  pq.emplace(0., start[0], start[1]);
  grid[start[0]][start[1]] = 0;
  std::vector v(size[0], std::vector<bool>(size[1], false));
  while (pq.size()) {
    auto [c, x, y] = pq.top();
    pq.pop();

    if (v[x][y]) continue;
    v[x][y] = true;

    for (int i = 0; i < 8; ++i) {
      auto [xx, yy] = std::pair{x + go[i][0], y + go[i][1]};
      if (xx >= 0 && xx < size[0] && yy >= 0 && yy < size[1]) {
        const double cost = i < 4 ? step : step * 1.414213;
        if (grid[xx][yy] > grid[x][y] + cost &&
            !om.IsAaBoxOccupied({xx * step - off, yy * step - off},
                                {xx * step + off, yy * step + off})) {
          grid[xx][yy] = grid[x][y] + cost;
          pq.push({grid[xx][yy], xx, yy});
        }
      }
    }
  }

  return grid;
}

std::vector<std::vector<double>> GenerateSpfaShortestPath2d(
    const AD2& base, const double step, const double width, const AI2& size,
    const OccupyMap& om, const AD2& from) {
  std::vector grid(size[0], std::vector<double>(size[1], 1e7));
  const AI2 start{static_cast<int>((from[0] - base[0]) / step),
                  static_cast<int>((from[1] - base[1]) / step)};
  const double off = width;

  std::array<AI2, 8> go{AI2{1, 0}, AI2{-1, 0}, AI2{0, 1},  AI2{0, -1},
                        AI2{1, 1}, AI2{-1, 1}, AI2{1, -1}, AI2{-1, -1}};
  std::queue<AI2> q{};
  std::vector v(size[0], std::vector<bool>(size[1], false));
  q.push(start);
  LOG_INFO("start: ({}, {}), grid: ({}, {})", start[0], start[1], size[0],
           size[1]);
  grid[start[0]][start[1]] = 0;
  v[start[0]][start[1]] = true;

  const double dir_cost = step, dia_cost = step * 1.414213;
  while (q.size()) {
    auto [x, y] = q.front();
    q.pop();
    v[x][y] = false;

    for (int i = 0; i < 8; ++i) {
      auto [xx, yy] = std::pair{x + go[i][0], y + go[i][1]};
      if (xx >= 0 && xx < size[0] && yy >= 0 && yy < size[1]) {
        const double cost = i < 4 ? dir_cost : dia_cost;
        if (grid[xx][yy] < grid[x][y] + cost ||
            om.IsAaBoxOccupied(
                {xx * step + base[0] - off, yy * step + base[1] - off},
                {xx * step + base[0] + off, yy * step + base[1] + off})) {
          continue;
        }
        grid[xx][yy] = grid[x][y] + cost;
        if (!v[xx][yy]) {
          v[xx][yy] = true;
          q.push({xx, yy});
        }
      }
    }
  }

  return grid;
}

std::vector<Node> GenerateNexts(const Config& conf, const Node& n) {
  std::vector<Node> ans{};
  const double str_max = conf.veh_max_steer;
  const double str_gap = 2 * conf.veh_max_steer / conf.node_steer_segs;
  const double step_arc = conf.node_arc_len / conf.node_arc_segs;
  auto sample = [&](int d) {
    for (double steer = -str_max + 1e-3; steer < str_max; steer += str_gap) {
      double tt = std::tan(steer);
      double angle = n.thetas.back() + d * step_arc / conf.veh_wheel_base * tt;
      Node curr{
          .xs = {n.xs.back() + d * step_arc * std::cos(angle)},
          .ys = {n.ys.back() + d * step_arc * std::sin(angle)},
          .thetas = {normalize_angle(angle +
                                     d * step_arc / conf.veh_wheel_base * tt)},
          .is_forward = d > 0,
          .steer = steer,
      };
      for (int i = 0; i < conf.node_arc_segs; ++i) {
        curr.xs.push_back(curr.xs[i] + d * step_arc * std::cos(curr.thetas[i]));
        curr.ys.push_back(curr.ys[i] + d * step_arc * std::sin(curr.thetas[i]));
        curr.thetas.push_back(normalize_angle(
            curr.thetas[i] + d * step_arc / conf.veh_wheel_base * tt));
      }
      ans.push_back(std::move(curr));
    }
  };
  sample(1);   // forward
  sample(-1);  // backward

  return ans;
}

bool IsBeyondTarget(const TaskInfo& task_info, const Node& cur,
                    const ReedsSheppPath::Point3d& end, double tar_s_bias) {
  SLPoint cur_pt, tar_pt;
  if (!task_info.reference_line()->GetPointInFrenetFrameWithHeading(
          {cur.xs.back(), cur.ys.back()}, cur.thetas.back(), &cur_pt) ||
      !task_info.reference_line()->GetPointInFrenetFrameWithHeading(
          {end.x, end.y}, end.phi, &tar_pt))
    return false;

  return cur.is_forward && (cur_pt.s() - tar_pt.s() >= tar_s_bias) &&
         (std::abs(cur_pt.l() - tar_pt.l()) < 0.5) &&
         (std::abs(cur.thetas.back() - end.phi) < 0.3);
}

bool IsRspCollisionFree(const OccupyMap& om, const ReedsSheppPath::Path& sp) {
  for (size_t i = 0; i < sp.x.size(); ++i) {
    if (om.IsVehicleBoxOccupied({sp.x[i], sp.y[i], sp.phi[i]})) return false;
  }
  return true;
}

bool FindNearestFeasibleEndPoint(const AD2& base, const AD2& max_pt,
                                 const double step, const OccupyMap& om,
                                 const AD3& to, const AI2& size,
                                 ReedsSheppPath::Point3d& near) {
  LOG_INFO("get in FindNearestFeasibleEndPoint");
  if (to[0] < base[0] || to[0] > max_pt[0] || to[1] < base[1] ||
      to[1] > max_pt[1]) {
    LOG_INFO("end point is out of boundary!");
    return false;
  }
  int go[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                  {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  const AI2 goal{static_cast<int>((to[0] - base[0]) / step),
                 static_cast<int>((to[1] - base[1]) / step)};
  std::vector grid(size[0], std::vector<double>(size[1], 1e7));
  std::vector<std::vector<bool>> visited(size[0],
                                         std::vector<bool>(size[1], false));
  using TDII = std::tuple<double, int, int>;
  std::priority_queue<TDII, std::vector<TDII>, std::greater<TDII>> pq{};
  pq.emplace(0.0, goal[0], goal[1]);

  LOG_INFO("Start FindNearestFeasibleEndPoint search");
  while (!pq.empty()) {
    auto [c, x, y] = pq.top();
    pq.pop();
    if (x < 0 || y < 0 || x >= size[0] || y >= size[1]) {
      LOG_INFO("current point is out of boundary!");
      continue;
    }

    visited[x][y] = true;
    AD3 cur = {x * step + base[0], y * step + base[1], to[2]};
    LOG_INFO("cur node xy: ({:.4f}, {:.4f}), idx: ({}, {})", cur[0], cur[1], x,
             y);
    if (!om.IsVehicleBoxOccupied(cur)) {
      near = {cur[0], cur[1], cur[2]};
      return true;
    }

    const double dir_cost = step, dia_cost = step * 1.414213;
    for (int i = 0; i < 8; ++i) {
      auto [xx, yy] = std::pair{x + go[i][0], y + go[i][1]};
      if (xx >= 0 && xx < size[0] && yy >= 0 && yy < size[1] &&
          !visited[xx][yy]) {
        double single_cost = i < 4 ? dir_cost : dia_cost;
        double total_cost = c + single_cost;
        pq.push({total_cost, xx, yy});
      }
    }
    if (pq.size() > 50) {
      LOG_INFO("pq oversize!");
      return false;
    }
  }
  return false;
}

bool ExtractPath(const std::vector<Node>& nodes, std::vector<int>& path) {
  if (nodes[1].parent == -1) {
    // No path is sampled due to collision detection failure at from node
    if (nodes.size() <= 2) {
      LOG_INFO("No path is sampled, Invalid query from start pt");
      return false;
    }
    // Paths were sampled, but not converging to end node
    LOG_WARN("end node not found");
    return false;
  } else {
    LOG_INFO("find the path!");
    for (int i = nodes[1].parent; i != -1; i = nodes[i].parent) {
      LOG_INFO("node steer = {}, {}", nodes[i].steer, nodes[i].is_forward);
      path.push_back(i);
    }
  }
  if (path.back() != 0) {
    LOG_WARN("start node not found");
    return false;
  }
  std::reverse(path.begin(), path.end());
  return true;
}

std::vector<PiecePath> BuildPiecePath(const std::vector<Node>& nodes,
                                      const std::vector<int>& path,
                                      const ReedsSheppPath::Path& sp,
                                      bool find_rsp) {
  std::vector<PiecePath> ans;
  double s = 0;

  auto fill_path = [](auto& path, auto x, auto y, auto theta, auto len) {
    path.xs.push_back(x), path.ys.push_back(y);
    path.thetas.push_back(theta), path.lens.push_back(len);
  };

  /// Build nodes
  const bool first_dir =
      path.size() > 1 ? nodes[path[1]].is_forward : sp.gear[0];
  ans.push_back({
      .is_forward = first_dir,
      .xs = {nodes[0].xs[0]},
      .ys = {nodes[0].ys[0]},
      .thetas = {nodes[0].thetas[0]},
      .lens = {0},
  });
  auto track = [](auto& pp) {
    return AD4{pp.xs.back(), pp.ys.back(), pp.thetas.back(), pp.lens.back()};
  };
  for (size_t i = 1; i < path.size(); ++i) {
    auto& curr = nodes[path[i]];
    if (ans.back().is_forward != curr.is_forward) {
      auto [px, py, pt, ps] = track(ans.back());
      ans.push_back({
          .is_forward = curr.is_forward,
          .xs = {px},
          .ys = {py},
          .thetas = {pt},
          .lens = {ps},
      });
    }
    for (size_t j = 0; j < curr.xs.size(); ++j) {
      auto [px, py, pt, ps] = track(ans.back());
      s += std::hypot(curr.xs[j] - px, curr.ys[j] - py);
      fill_path(ans.back(), curr.xs[j], curr.ys[j], curr.thetas[j], s);
    }
  }

  /// Build ReedsShepp
  if (find_rsp) {
    if (sp.gear[0] != ans.back().is_forward) {
      auto [px, py, pt, ps] = track(ans.back());
      // const double af = sp.gear[0] ? 0 : M_PI;
      ans.push_back({
          .is_forward = sp.gear[0],
          .xs = {sp.x[0]},
          .ys = {sp.y[0]},
          .thetas = {sp.phi[0]},
          .lens = {ps},
      });
    }
    for (size_t i = 1; i < sp.x.size(); ++i) {
      auto [px, py, pt, ps] = track(ans.back());
      if (ans.back().is_forward != sp.gear[i]) {
        ans.push_back({
            .is_forward = sp.gear[i],
            .xs = {px},
            .ys = {py},
            .thetas = {pt},
            .lens = {ps},
        });
      }
      s += std::hypot(sp.x[i] - px, sp.y[i] - py);
      fill_path(ans.back(), sp.x[i], sp.y[i], sp.phi[i], s);
    }
  }
  return ans;
}

}  // namespace

HybridAStar::HybridAStar(HybridAStar::Config&& conf) : conf_{conf} {}

std::vector<PiecePath> HybridAStar::GeneratePath(const OccupyMap& om,
                                                 const AD3& from, const AD3& to,
                                                 TaskInfo& task_info) {
  auto start_time = cyber::Time::Now().ToSecond();
  auto end_time = cyber::Time::Now().ToSecond();
  auto data = task_info.current_frame()->mutable_inside_planner_data();
  static ReedsSheppPath rsp{
      {.max_kappa = std::tan(conf_.veh_max_steer * 0.75) / conf_.veh_wheel_base,
       .step_size = 0.1}};
  auto [minx, miny, maxx, maxy] = om.GetRange();
  AI2 size{static_cast<int>((maxx - minx) / conf_.grid_step + 1),
           static_cast<int>((maxy - miny) / conf_.grid_step + 1)};
  minx_ = minx;
  miny_ = miny;
  size_ = size;
  start_time = cyber::Time::Now().ToSecond();
  LOG_INFO(
      "base: ({:.4f}, {:.4f}), max_xy: ({:.4f}, {:.4f}), end: ({:.4f}, {:.4f})",
      minx, miny, maxx, maxy, to[0], to[1]);
  if (to[0] < minx || to[1] < miny || to[0] > maxx || to[1] > maxy ||
      (to[0] - minx) / conf_.grid_step > size[0] ||
      (to[1] - miny) / conf_.grid_step > size[1]) {
    LOG_INFO("oversize! base: ({:.4f}, {:.4f}), end: ({:.4f}, {:.4f})", minx,
             miny, to[0], to[1]);
    return {};
  }
  // collision check of start and target pt
  if (om.IsVehicleBoxOccupied(from)) {
    LOG_INFO("Invalid query from ({}, {}, {})", from[0], from[1], from[2]);
    return {};
  }

  ReedsSheppPath::Point3d end_pt{to[0], to[1], to[2]};
  if (om.IsVehicleBoxOccupied(to)) {
    LOG_INFO("target pt({:.4f}, {:.4f}, {:.4f}) is occupied", end_pt.x,
             end_pt.y, end_pt.phi);
    if (!FindNearestFeasibleEndPoint({minx, miny}, {maxx, maxy},
                                     conf_.grid_step, om, to, size, end_pt)) {
      LOG_WARN("no feasible pt is found!");
      return {};
    }
    LOG_INFO("nearest feasible pt({:.4f}, {:.4f}, {:.4f}) is found!", end_pt.x,
             end_pt.y, end_pt.phi);
  }

  heu_map_ = GenerateSpfaShortestPath2d({minx, miny}, conf_.grid_step,
                                        conf_.veh_half_width, size, om,
                                        {end_pt.x, end_pt.y});
  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("GenerateSpfaShortestPath2d XXX use time: {:.4f}",
           end_time - start_time);
  // start 0 node; end 1 node
  nodes_ = {
      {
          .idx = 0,
          .xs = {from[0]},
          .ys = {from[1]},
          .thetas = {from[2]},
          .go_cost = 0,
          .heu_cost = 0,
      },
      {
          .idx = 1,
          .xs = {end_pt.x},
          .ys = {end_pt.y},
          .thetas = {end_pt.phi},
      },
  };

  std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq{};
  std::unordered_set<int> st{};
  pq.push({0, 0});
  std::vector node_map(size[0], std::vector<double>(size[1], 1e9));
  node_map[(from[0] - minx) / conf_.grid_step]
          [(from[1] - miny) / conf_.grid_step] = 0;

  auto is_collision_free = [](auto& node, auto& om) -> bool {
    for (size_t i = 0; i < node.xs.size(); ++i) {
      if (om.IsVehicleBoxOccupied({node.xs[i], node.ys[i], node.thetas[i]}))
        return false;
    }
    return true;
  };

  int step = 0;
  ReedsSheppPath::Path sp{};
  start_time = cyber::Time::Now().ToSecond();
  bool find_rsp = false;

  while (pq.size()) {
    auto [cost, curr] = pq.top();
    pq.pop();

    // Termination of search: 1.The trajectory exceeds the planning target
    if (IsBeyondTarget(task_info, nodes_[curr], end_pt, conf_.tar_s_bias)) {
      LOG_INFO("XXX Found target by distance");
      nodes_[1].parent = curr;
      break;
    }
    // 2.Find the RS curve
    if ((step++ % conf_.node_one_shot_gap == 0) &&
        rsp.ShortestRsp({nodes_[curr].xs.back(), nodes_[curr].ys.back(),
                         nodes_[curr].thetas.back()},
                        end_pt, &sp) &&
        IsRspCollisionFree(om, sp) && sp.gear[0]) {
      LOG_INFO("XXX Found target by RSP {}", step);
      nodes_[1].parent = curr;
      find_rsp = true;
      break;
    }

    if (st.count(curr)) continue;
    st.insert(curr);

    for (auto& node : GenerateNexts(conf_, nodes_[curr])) {
      // Once go forward, never go back any more
      if (curr != 0 && nodes_[curr].is_forward && !node.is_forward) continue;
      // Avoid the steer changing from zero to max and collision
      if ((std::abs(nodes_[curr].steer) < 0.01 &&
           std::abs(node.steer) > 0.35) ||
          !is_collision_free(node, om))
        continue;

      const int nx = (node.xs.back() - minx) / conf_.grid_step;
      const int ny = (node.ys.back() - miny) / conf_.grid_step;
      const double g = nodes_[curr].go_cost +
                       GoCost(nodes_[curr], node, data->left_pass_obs);
      const double h = 50 * heu_map_[nx][ny];
      if (node_map[nx][ny] > g + h) {
        node.idx = static_cast<int>(nodes_.size());
        node.go_cost = g;
        node.heu_cost = h;
        node.parent = curr;
        nodes_.push_back(node);

        node_map[nx][ny] = g + h;
        pq.push({node_map[nx][ny], node.idx});

        if (step <= 10)
          LOG_INFO("cur {}, id {}, steer {:.1f}, cost {:.4f}, dir {}", curr,
                   node.idx, node.steer, g + h, node.is_forward);
      }
    }
    if (pq.size() > 1e5) {
      LOG_WARN("Node size {} is too bigger", pq.size());
      break;
    }
  }
  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("ReedsSheppPath XXX use time: {:.4f}", end_time - start_time);
  LOG_INFO("ReedsSheppPath XXX step: {}", step);

  std::vector<int> path{};
  if (!ExtractPath(nodes_, path)) {
    LOG_INFO("ExtractPath fails, quit");
    return {};
  }

  start_time = cyber::Time::Now().ToSecond();
  std::vector<PiecePath> ans = BuildPiecePath(nodes_, path, sp, find_rsp);

  end_time = cyber::Time::Now().ToSecond();
  LOG_INFO("PiecePath XXX use time: {:.4f}", end_time - start_time);
  return ans;
}

}  // namespace planning
}  // namespace neodrive
