#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "math.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/math/common/geometry.h"

namespace neodrive {
namespace planning {
namespace level_k_context {
using planning::math::Polygon;
struct State {
  double x{};
  double y{};
  double v{};
  double theta{};
  double a{};
  double omeca{};
  std::vector<double> weights = {1.0, 1.0, 2.0, 1.0};
  State() = default;
  State(const double c_x, const double c_y, const double speed,
        const double heading, const double acc, const double o)
      : x(c_x), y(c_y), v(speed), theta(heading), a(acc), omeca(o){};

  void Reset() {
    x = 0.0;
    y = 0.0;
    v = 0.0;
    theta = 0.0;
    a = 0.0;
    omeca = 0.0;
  }

  bool operator==(const State& s) const {
    return s.x == x && s.y == y && s.v == v && s.a == a && s.theta == theta &&
           s.omeca == omeca;
  }

  double operator-(const State& other) {
    return weights[0] * std::abs(x - other.x) +
           weights[1] * std::abs(y - other.y) +
           weights[2] * std::abs(v - other.v) +
           weights[3] * std::abs(theta - other.theta);
  }
};

struct TrajPoint {
  double t;
  double x;
  double y;
  double theta;
  double s;
  double v;
  double a;
  void Reset(const State& state, const size_t time_stamp) {
    t = time_stamp;
    x = state.x;
    y = state.y;
    theta = state.theta;
    s = 0.0;
    v = state.v;
    a = state.a;
  }

  bool operator==(const TrajPoint& pt) const {
    return pt.t == t && pt.x == x && pt.y == y && pt.theta == theta;
  }
};

struct Trajectory {
  std::vector<TrajPoint> pts;
  Trajectory() = default;
  void Reset(const State& state) {
    TrajPoint init_pt;
    init_pt.Reset(state, 0);
    pts.emplace_back(init_pt);
  }

  void Reset() { pts.clear(); }

  bool operator==(const Trajectory& traj) const { return traj.pts == pts; }
};

struct Param {
  double width{};
  double length{};
  double front_suspension{};
  double rear_suspension{};
  double max_steering_angle{};
  double max_longitudinal_acc{};
  Param() {}
  Param(const double w, const double l, const double f_s, const double r_s,
        const double m_h, const double m_a)
      : width(w),
        length(l),
        front_suspension(f_s),
        rear_suspension(r_s),
        max_steering_angle(m_h),
        max_longitudinal_acc(m_a) {}
  void Reset() {
    width = 8.0;
    length = 4.0;
    front_suspension = 2.29;
    rear_suspension = 0.4;
    max_steering_angle = 6.6323;
    max_longitudinal_acc = 2.0;
  }
};

struct Vehicle;

struct Believes {
  std::vector<double> believes = {1.0, 0.0, 0.0};
  std::vector<std::vector<Vehicle>> trajs =
      std::vector<std::vector<Vehicle>>(3);
  size_t k = 0;
  void set_belief(const std::vector<double>& b, const std::vector<Vehicle>& t0,
                  const std::vector<Vehicle>& t1,
                  const std::vector<Vehicle>& t2) {
    if (b.size() == believes.size()) {
      believes = b;
    }
    trajs[0] = t0;
    trajs[1] = t1;
    trajs[2] = t2;
    auto maxElement = std::max_element(believes.begin(), believes.end());
    if (maxElement != believes.end()) {
      k = std::distance(believes.begin(), maxElement);
    }
  }
  void Reset() {
    k = 0;
    believes = {1.0, 0.0, 0.0};
    trajs = std::vector<std::vector<Vehicle>>(3, std::vector<Vehicle>());
  }
  bool operator==(const Believes& b) const {
    return b.believes == believes && b.k == k && b.trajs == trajs;
  }
};

struct OrinTrajectory {
  double probability = 0.0;
  Trajectory orin_traj{};
};

struct Vehicle {
  int id{};
  State state;
  Param param;
  Polygon polygon;
  std::vector<Vehicle> traj;
  Trajectory goal_traj;
  Believes believe;
  size_t max_idxs = 0;
  Vec2d goal_pt{};
  double goal_s{};
  std::vector<Vec2d> ref_pts{};
  std::vector<OrinTrajectory> orin_trajs{};

  Vehicle() {}

  Vehicle(const int i, const State& s, const Param& p, const Believes& b,
          const Vec2d& g_pt, const double g_s, const std::vector<Vec2d>& r_pt)
      : id(i),
        state(s),
        param(p),
        believe(b),
        goal_pt(g_pt),
        goal_s(g_s),
        ref_pts(r_pt) {
    goal_traj.Reset(s);
  }

  void SetParam(const int i, const State& s, const Param& p, const Believes& b,
                const Vec2d& g_pt, const double g_s,
                const std::vector<Vec2d>& r_pt) {
    id = i;
    state = s;
    param = p;
    believe = b;
    goal_pt = g_pt;
    goal_s = g_s;
    ref_pts = r_pt;
    goal_traj.Reset(s);
  }

  void SetPredTraj(const std::vector<OrinTrajectory>& traj) {
    orin_trajs.clear();
    orin_trajs = traj;
    max_idxs = traj.size();
  }

  void Reset() {
    id = 0;
    goal_pt = Vec2d(0.0, 0.0);
    goal_s = 0.0;
    state.Reset();
    param.Reset();
    goal_traj.Reset();
    ref_pts.clear();
  }

  bool operator==(const Vehicle& v) const {
    return v.id == id && v.state == state && v.believe == believe;
  }
};

struct Actions {
  int size_actions = 5;
  std::map<std::string, std::tuple<double, double>> action_set = {
      {"Acc", std::make_tuple(2.5, 0.0)},
      {"Dec", std::make_tuple(-2.5, 0.0)},
      {"Maintain", std::make_tuple(0.0, 0.0)},
      {"Turnleft", std::make_tuple(0.0, M_PI_2 / 2.0)},
      {"Turnright", std::make_tuple(0.0, M_PI_2 / 2.0)}};
};

struct Weights {
  double w_czone{200.0};
  double w_szone{20.0};
  double w_offroad{100.0};
  double w_retrograde{10.0};
  double w_goaldist{1.0};
  double w_ref{5.0};
};

struct CrossCheckData {
  std::unordered_map<std::string, double> accumulate_level_cost{
      {"zero", 0.0}, {"one", 0.0}, {"two", 0.0}};

  std::unordered_map<std::string, double> accumulate_cost_zero_vs{
      {"zero", 0.0}, {"one", 0.0}, {"two", 0.0}};
  std::unordered_map<std::string, double> accumulate_cost_one_vs{
      {"zero", 0.0}, {"one", 0.0}, {"two", 0.0}};
  std::unordered_map<std::string, double> accumulate_cost_two_vs{
      {"zero", 0.0}, {"one", 0.0}, {"two", 0.0}};

  std::unordered_map<std::string, std::vector<double>> costs_zero_vs{
      {"zero", std::vector<double>()},
      {"one", std::vector<double>()},
      {"two", std::vector<double>()}};
  std::unordered_map<std::string, std::vector<double>> costs_one_vs{
      {"zero", std::vector<double>()},
      {"one", std::vector<double>()},
      {"two", std::vector<double>()}};
  std::unordered_map<std::string, std::vector<double>> costs_two_vs{
      {"zero", std::vector<double>()},
      {"one", std::vector<double>()},
      {"two", std::vector<double>()}};
};

class Node {
 public:
  Node(const Vehicle& data)
      : data_(data), children_(size_branch_, nullptr), father_(nullptr) {}

  Node() {}

  bool insert(const int index, Node* node) {
    if (index < 0 || index >= 9) {
      return false;
    }
    children_[index] = node;
    return true;
  }

  std::vector<Vehicle> dfs_traversal() {
    std::vector<Vehicle> result{};
    result.emplace_back(data_);
    for (Node* child : children_) {
      if (child != nullptr) {
        std::vector<Vehicle> child_result = child->dfs_traversal();
        result.insert(result.end(), child_result.begin(), child_result.end());
      }
    }
    return result;
  }

 public:
  Actions actions_;
  Vehicle data_;
  int size_branch_{actions_.size_actions};
  std::vector<Node*> children_{};
  Node* father_;
};
}  // namespace level_k_context
}  // namespace planning
}  // namespace neodrive