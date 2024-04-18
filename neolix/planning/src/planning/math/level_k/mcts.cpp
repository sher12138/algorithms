#include "mcts.h"

namespace neodrive {
namespace planning {
namespace mcts_u {
bool StateTrans(level_k_context::Vehicle& agent, const double action,
                const double t, const size_t idx) {
  if (agent.goal_traj.pts.empty()) {
    LOG_WARN("mcts failed: initial traj is empty!");
    return false;
  }
  auto cur_pt = agent.goal_traj.pts.back();
  double next_t = t;
  double next_a = action;
  double next_v = (cur_pt.v >= -action * 0.1) ? cur_pt.v + action * 0.1 : 0.0;
  double step_s = (cur_pt.v >= -action * 0.1)
                      ? cur_pt.v * 0.1 + 0.5 * action * 0.01
                      : std::pow(cur_pt.v, 2) / (2.0 * action);
  double next_s = cur_pt.s + step_s;
  std::vector<double> svat = {next_s, next_v, next_a, next_t};
  level_k_context::TrajPoint new_pt;
  if (!StepByBinary(agent.orin_trajs[idx].orin_traj, svat, new_pt)) {
    LOG_INFO("Calc step by binary failed!");
    return false;
  }
  agent.goal_traj.pts.emplace_back(new_pt);
  return true;
}

bool StateTrans(level_k_context::Vehicle& agent, const size_t idx) {
  if (agent.goal_traj.pts.empty()) {
    LOG_WARN("mcts failed: initial traj is empty!");
    return false;
  }
  auto cur_pt = agent.goal_traj.pts.back();
  double next_t = 0.0;
  double next_a = cur_pt.a;
  double next_v = cur_pt.v;
  double next_s = 0.0;
  std::vector<double> svat = {next_s, next_v, next_a, next_t};
  level_k_context::TrajPoint new_pt;
  if (!StepByBinary(agent.orin_trajs[idx].orin_traj, svat, new_pt)) {
    LOG_INFO("Calc step by binary failed!");
    return false;
  }
  agent.goal_traj.pts.emplace_back(new_pt);
  return true;
}

bool StepByBinary(const level_k_context::Trajectory& orin_traj,
                  const std::vector<double>& svat,
                  level_k_context::TrajPoint& new_pt) {
  if (orin_traj.pts.empty()) {
    return false;
  }
  if (orin_traj.pts.back().s < svat[0]) {
    LOG_INFO("Pred traj is too short!");
    return false;
  }
  new_pt.s = svat[0];
  new_pt.v = svat[1];
  new_pt.t = svat[3];
  auto binary_search = [](const level_k_context::Trajectory& traj,
                          const double s) -> int {
    int l = 0;
    int r = traj.pts.size() - 1;
    int prevIdx = -1;

    while (l <= r) {
      int m = l + (r - l) / 2;

      if (traj.pts[m].s == s) {
        return m;
      } else if (traj.pts[m].s < s) {
        prevIdx = m;
        l = m + 1;
      } else {
        r = m - 1;
      }
    }
    return prevIdx;
  };

  auto inter_value = [](const level_k_context::TrajPoint& l_pt,
                        const level_k_context::TrajPoint& r_pt,
                        const double ratio,
                        level_k_context::TrajPoint& tar_pt) {
    tar_pt.x = l_pt.x + ratio * (r_pt.x - l_pt.x);
    tar_pt.y = l_pt.y + ratio * (r_pt.y - l_pt.y);
    tar_pt.theta = l_pt.theta + ratio * (r_pt.theta - l_pt.theta);
  };

  int idx = binary_search(orin_traj, svat[0]);
  level_k_context::TrajPoint l_pt = orin_traj.pts[idx];
  level_k_context::TrajPoint r_pt =
      (idx < orin_traj.pts.size() - 1) ? orin_traj.pts[idx + 1] : l_pt;
  double ratio = (idx < orin_traj.pts.size() - 1)
                     ? (svat[0] - l_pt.s) / (r_pt.s - l_pt.s)
                     : 0.0;
  inter_value(l_pt, r_pt, ratio, new_pt);
  return true;
}

void MctsUtils::MonteCarloTreeSearch(Node* best_next_node) {
  int computation_budget = 1000;
  for (int i = 0; i < computation_budget; ++i) {
    Node expend_node = TreePolicy(node_);
    double r = DefaultPolicy(expend_node);
    BackUp(&expend_node, r);
  }
  BestChild(node_, true, best_next_node);
}

void MctsUtils::MonteCarloTreeSearch(
    std::vector<level_k_context::Vehicle>& best_traj) {
  int computation_budget = 1000;
  for (int i = 0; i < computation_budget; ++i) {
    Node expend_node = TreePolicy(node_);
    double r = DefaultPolicy(expend_node);
    BackUp(&expend_node, r);
  }
  Node best_next_node;
  BestChild(node_, true, &best_next_node);
  while (!best_next_node.get_children().empty()) {
    best_traj.emplace_back(best_next_node.get_state().ego);
    BestChild(best_next_node, false, &best_next_node);
  }
}

void MctsUtils::BestChild(Node& node, const bool is_exploration,
                          Node* best_subnode) {
  double best_score = -std::numeric_limits<double>::max();
  for (const auto& sub_node : node.get_children()) {
    double c = 0.0;
    if (is_exploration) {
      c = 1.0 / std::sqrt(2.0);
    } else {
      c = 0.0;
    }
    double left = sub_node.get_quality_value() / sub_node.get_visit_times();
    double right =
        2.0 * std::log(node.get_visit_times()) / sub_node.get_visit_times();
    double score = left + c * std::sqrt(right);
    if (score > best_score) {
      best_score = score;
      *best_subnode = sub_node;
    }
  }
}

void MctsUtils::Expand(Node& node, Node* subnode) {
  std::vector<State> tried_sub_node_states{};
  for (const auto& n : node.get_children()) {
    tried_sub_node_states.emplace_back(n.get_state());
  }
  State new_state;
  node.get_state().get_next_state_with_random_choice(
      node.get_state().get_t(), node.get_state().get_idx(), &new_state);
  while (std::find(tried_sub_node_states.begin(), tried_sub_node_states.end(),
                   new_state) != tried_sub_node_states.end()) {
    node.get_state().get_next_state_with_random_choice(
        node.get_state().get_t(), node.get_state().get_idx(), &new_state);
  }
  subnode->set_state(new_state);
  node.add_child(*subnode);
}

Node MctsUtils::TreePolicy(Node& node) {
  while (!node.get_state().IsTerminal()) {
    if (node.is_all_expand()) {
      BestChild(node, true, &node);
    } else {
      Node subnode;
      Expand(node, &subnode);
      return subnode;
    }
  }
  return node;
}

double MctsUtils::DefaultPolicy(const Node& node) {
  State current_state = node.get_state();
  while (!current_state.IsTerminal()) {
    current_state.get_next_state_with_random_choice(
        node.get_state().get_t(), node.get_state().get_idx(), &current_state);
  }
  return current_state.get_reward();
}

void MctsUtils::BackUp(Node* node, const double reward) {
  while (node != nullptr) {
    node->visit_times_add_one();
    node->quality_value_add_n(reward);
    node = node->parent;
  }
}

}  // namespace mcts_u
}  // namespace planning
}  // namespace neodrive