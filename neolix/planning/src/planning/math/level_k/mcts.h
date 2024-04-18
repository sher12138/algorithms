#pragma once
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <random>
#include <vector>

#include "context.h"
#include "src/planning/math/common/geometry.h"
#include "src/planning/math/common/temporal_solver.h"

namespace neodrive {
namespace planning {
namespace mcts_u {
using planning::obsmap::TrajectoryData;
bool StateTrans(level_k_context::Vehicle& agent, const double action,
                const double t, const size_t idx);

bool StateTrans(level_k_context::Vehicle& agent, const size_t idx);

template <typename T>
T randomSelect(const std::vector<T>& list) {
  std::mt19937 gen(static_cast<unsigned int>(std::time(nullptr)));
  std::uniform_int_distribution<> dist(0, static_cast<int>(list.size()) - 1);
  int index = dist(gen);
  return list[index];
};

bool StepByBinary(const level_k_context::Trajectory& orin_traj,
                  const std::vector<double>& svat,
                  level_k_context::TrajPoint& new_pt);

struct State {
  double value = -1e+3;
  double t = 0.0;
  int idx = -1;
  int current_round_index = 0;
  std::vector<std::vector<double>> cumulative_choices{};
  level_k_context::Vehicle
      ego;  // note: The definitions of ego and agent can be reversed
  std::vector<level_k_context::Vehicle> agents{};
  std::vector<std::vector<level_k_context::Vehicle>> agents_set{};
  double action = 0.0;
  bool is_collision = false;
  double Refficiency = 0.5;
  double Rcomfort = -0.5;
  double Rsafe = -100.0;
  double Rgoal = 10.0;

  State() = default;

  bool operator==(const State& s) const {
    return s.ego == ego && s.agents == agents;
  }

  bool IsTerminal() {
    int MAX_ROUND_NUMBER = 200;  // todo (lvyang): tmp
    return (current_round_index == MAX_ROUND_NUMBER - 1 || is_collision ||
            ego.goal_s < ego.goal_traj.pts.back().s);
  }

  // multi agents collision check
  bool CalCollision(const double frame_id, const level_k_context::Vehicle& e,
                    const std::vector<level_k_context::Vehicle>& as) {
    if (as.empty()) {
      return false;
    }
    auto trans_data = [](const double frame_id,
                         const level_k_context::Vehicle& v, TrajectoryData& d) {
      d.id = v.id;
      d.timestamp.emplace_back(frame_id);
      d.position.emplace_back(math::AD2{v.state.x, v.state.y});
      d.heading.emplace_back(v.state.theta);
      d.polygon.emplace_back(v.polygon);
    };
    std::vector<TrajectoryData> datas{};
    TrajectoryData data_ego;
    trans_data(frame_id, e, data_ego);
    datas.emplace_back(data_ego);
    for (size_t i = 0; i < as.size(); ++i) {
      level_k_context::Vehicle a = as[i];
      TrajectoryData data_agent;
      trans_data(i * 0.1, a, data_agent);
      datas.emplace_back(data_agent);
    }
    obsmap::TemporalCollision tpc;
    tpc.PreProcess(datas);
    std::vector<const math::Node<math::Polygon>*> c_polygons =
        tpc.DetectCollisionAt(frame_id, e.polygon);
    return !c_polygons.empty();
  }

  // multi agents reward calc
  double CalReward(const double cur_t, const level_k_context::Vehicle& e,
                   const std::vector<level_k_context::Vehicle>& as) {
    double reward = 0.0;
    double step_s =
        (e.goal_traj.pts.back().v + action * 0.1 >= 0.0)
            ? e.goal_traj.pts.back().v * 0.1 + 0.5 * action * 0.01
            : -std::pow(e.goal_traj.pts.back().v, 2) / (2.0 * action);
    is_collision = CalCollision(cur_t, e, as);
    reward += Refficiency * step_s;
    reward += Rcomfort * std::abs(action);
    reward += Rsafe * (is_collision ? 1.0 : 0.0);
    reward += Rgoal * (e.goal_s < e.goal_traj.pts.back().s ? 1.0 : 0.0);
    return reward;
  }

  // follow-up step
  void Step(const double cur_t, const int idx, State& next_state, double* val) {
    StateTrans(next_state.ego, action, cur_t + 0.1, idx);
    double tmp_value =
        CalReward(cur_t + 0.1, next_state.ego, next_state.agents);
    next_state.agents =
        next_state.agents_set[static_cast<int>(next_state.t / 0.1)];
    *val = tmp_value;
  }

  // first layer step
  void Step(const int idx, State& next_state, double* val) {
    StateTrans(next_state.ego, idx);
    next_state.agents =
        next_state.agents_set[static_cast<int>(next_state.t / 0.1)];
    *val = 0.0;
  }

  void set_current_ego_state(const level_k_context::Vehicle& v) { ego = v; }

  void set_current_agents_state(
      const std::vector<level_k_context::Vehicle>& vs) {
    agents = vs;
  }

  void set_current_agents_state(const level_k_context::Vehicle& v) {
    agents.clear();
    agents.emplace_back(v);
  }

  void set_agents_set(
      const std::vector<std::vector<level_k_context::Vehicle>>& ass) {
    agents_set = ass;
  }

  void set_agents_set(const std::vector<level_k_context::Vehicle>& as) {
    agents_set.clear();
    agents_set.emplace_back(as);
  }

  void set_agents_set(const level_k_context::Vehicle& a) {
    agents_set.clear();
    agents_set.emplace_back(std::vector<level_k_context::Vehicle>(100, a));
  }

  level_k_context::Vehicle get_current_ego_state() { return ego; }

  std::vector<level_k_context::Vehicle> get_current_agents_state() {
    return agents;
  }

  void set_current_round_index(const int round) { current_round_index = round; }

  void set_cumulative_choise(const std::vector<std::vector<double>>& choices) {
    cumulative_choices = choices;
  }

  void set_idx(const int index) { idx = index; };

  void get_next_state_with_random_choice(const double cur_t, const int idx,
                                         State* next_state) {
    std::vector<double> AVAILABLE_CHOICES = {-1.0, -2.0, 1.0, 2.0, 0.0};
    std::vector<int> IDX_CHOICES{};
    for (size_t i = 0; i < ego.max_idxs; ++i) {
      IDX_CHOICES.emplace_back(i);
    }
    int AVAILABLE_CHOICE_NUMBER =
        AVAILABLE_CHOICES.size();  // todo (lvyang): tmp
    double random_choice = randomSelect(AVAILABLE_CHOICES);
    int random_idx = randomSelect(IDX_CHOICES);
    action = random_choice;
    if (cur_t < 0.1) {
      Step(idx, *next_state, &(next_state->value));
      next_state->t = cur_t;
      next_state->set_idx(random_idx);
    } else {
      Step(cur_t, idx, *next_state, &(next_state->value));
      next_state->t = cur_t + 0.1;
      next_state->set_idx(idx);
      next_state->set_current_round_index(current_round_index + 1);
      std::vector<std::vector<double>> next_cumulative_choices{};
      for (const auto& iter : cumulative_choices) {
        next_cumulative_choices.emplace_back(iter);
      }
      next_cumulative_choices.emplace_back(std::vector<double>(random_choice));
      next_state->set_cumulative_choise(next_cumulative_choices);
    }
  }

  double get_reward() const { return value; }

  double get_action() const { return action; }

  double get_t() const { return t; }

  int get_idx() const { return idx; }
};

struct Node {
  Node* parent = nullptr;
  std::vector<Node> children{};
  int visit_times = 0;
  double quality_value = 0.0;
  State state;
  bool is_terminal = false;

  void set_state(const State& s) { state = s; }

  State get_state() const { return state; }

  void set_parent(Node* p) { parent = p; }

  void get_parent(Node* p) const { p = parent; }

  void set_children(const std::vector<Node>& c) { children = c; }

  std::vector<Node> get_children() const { return children; }

  int get_visit_times() const { return visit_times; }

  void set_visit_times(int times) { visit_times = times; }

  void visit_times_add_one() { visit_times += 1; }

  double get_quality_value() const { return quality_value; }

  void set_quality_value(const double v) { quality_value = v; }

  void quality_value_add_n(const double n) { quality_value += n; }

  bool is_all_expand() {
    std::vector<double> AVAILABLE_CHOICES = {-1.0, -2.0, 1.0, 2.0, 0.0};
    int AVAILABLE_CHOICE_NUMBER =
        AVAILABLE_CHOICES.size();  // todo (lvyang): tmp
    if (children.size() == AVAILABLE_CHOICE_NUMBER) {
      return true;
    }
    return false;
  }

  bool is_all_expand_layer() {
    if (children.size() == state.ego.max_idxs) {
      return true;
    }
    return false;
  }

  void add_child(Node& sub_node) {
    sub_node.set_parent(parent);
    children.emplace_back(sub_node);
  }

  bool check_terminal() {
    is_terminal = state.IsTerminal();
    return is_terminal;
  }
};

class MctsUtils {
 public:
  MctsUtils() = default;

  MctsUtils(const size_t max_idxs, Node& node)
      : max_idxs_(max_idxs), node_(node) {}

  void MonteCarloTreeSearch(Node* best_next_node);

  void MonteCarloTreeSearch(std::vector<level_k_context::Vehicle>& best_traj);

 private:
  void BestChild(Node& node, const bool is_exploration, Node* best_subnode);

  void Expand(Node& node, Node* subnode);

  Node TreePolicy(Node& node);

  double DefaultPolicy(const Node& node);

  void BackUp(Node* node, const double reward);

 private:
  size_t max_idxs_ = 0;
  Node node_;
};
}  // namespace mcts_u

}  // namespace planning
}  // namespace neodrive