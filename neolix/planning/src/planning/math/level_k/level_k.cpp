#include "level_k.h"

namespace neodrive {
namespace planning {
bool LevelK::CalcLocalOptimalTrajectory() {
  if (agents_.empty()) {
    LOG_INFO("There aren't sounding obstacles!");
    return true;
  }
  for (auto& agent : agents_) {
    // temporarily double loop: update agents believes
    for (auto& ls : last_states_) {
      if (agent.id == ls.id) {
        if (!UpdateBelief(ls.believe.trajs, agent)) {
          LOG_INFO("Update belief failed, agent id is {}", agent.id);
          continue;
        }
      }
    }
    std::vector<Vehicle> winner_stategy{};
    Trajectory traj;
    if (!MakeEgoCopingStrategy(agent, ego_, winner_stategy, traj)) {
      LOG_WARN("Agent's local optimal strategy calc failed, id is {}!",
               agent.id);
      continue;
    }
    agent.goal_traj = traj;
    agent.traj = winner_stategy;
  }
  return true;
}

bool LevelK::CalcGlobalOptimalTrajectory() {
  if (agents_.empty()) {
    LOG_INFO("There aren't sounding obstacles!");
    return true;
  }
  if (!CalcLocalOptimalTrajectory()) {
    LOG_INFO("Calc local optimal traj failed!");
    return false;
  }
  std::vector<Vehicle> winner_traj{};
  if (!CalWinnerTrajectory(ego_, agents_, winner_traj)) {
    LOG_ERROR("Calc globle winner traj failed!");
    return false;
  }
  TransVehSetToTraj(winner_traj, ego_.goal_traj);
  return true;
}

double LevelK::CostFunction(const Vehicle& ego, const Vehicle& agent,
                            const bool& off_road_flag,
                            const bool& retrograde_flag, const Vec2d& goal_pt,
                            const std::vector<Vec2d>& refline_pts,
                            const Vec2d& search_pt) {
  std::vector<Vec2d> ego_vertice_without_safe{}, agent_vertice_without_safe{};
  std::vector<Vec2d> ego_vertice_with_safe{}, agent_vertice_with_safe{};
  double czone_area = level_k_utils::CalOverlap(ego_vertice_without_safe,
                                                agent_vertice_without_safe);
  double czone = (czone_area > 0.0) ? -1.0 : 0.0;
  double szone_area =
      level_k_utils::CalOverlap(ego_vertice_with_safe, agent_vertice_with_safe);
  double szone = (szone_area > 0.0) ? -1.0 : 0.0;
  double off_road = off_road_flag ? -1.0 : 0.0;
  double retrograde = retrograde_flag ? -1.0 : 0.0;
  double goal_dist = std::sqrt(std::pow(goal_pt.x() - search_pt.x(), 2) +
                               std::pow(goal_pt.y() - search_pt.y(), 2));
  double min_dist = level_k_utils::SearchNearestPtDist(search_pt, refline_pts);
  double cost = weights_.w_czone * czone + weights_.w_szone * szone +
                weights_.w_offroad * off_road +
                weights_.w_retrograde * retrograde +
                weights_.w_goaldist * goal_dist - weights_.w_ref * min_dist;
  return cost;
}

double LevelK::CostFunction(const std::vector<Vehicle>& traj_ego,
                            const std::vector<Vehicle>& traj_agent,
                            const Vec2d& goal_pt, const double goal_s,
                            const Vec2d& search_pt) {
  bool is_collision = level_k_utils::CalCollision(traj_ego, traj_agent);
  double czone = is_collision ? -1.0 : 0.0;
  bool buffer_collision = level_k_utils::CalCollision(traj_ego, traj_agent);
  double szone = buffer_collision ? -1.0 : 0.0;
  double goal_dist = std::sqrt(std::pow(goal_pt.x() - search_pt.x(), 2) +
                               std::pow(goal_pt.y() - search_pt.y(), 2));
  // double min_dist = level_k_utils::SearchNearestPtDist(search_pt,
  // refline_pts);
  double cost = weights_.w_czone * czone + weights_.w_szone * szone +
                weights_.w_goaldist * goal_dist;
  return cost;
}

double LevelK::AccumulateCost(const double cost, const int n) {
  return std::pow(lambda_, n) * cost;
}

bool LevelK::UpdateBelief(std::vector<std::vector<Vehicle>>& last_planning,
                          Vehicle& cur_state) {
  int k = last_planning.size();
  std::map<int, double> k_star_set;

  for (int i = 0; i < k; i++) {
    double level_i_norm = level_k_utils::CalDeltaState(
        last_planning[i][1].state, cur_state.state);
    k_star_set[i] = level_i_norm;
  }

  std::map<int, double> sorted_k_star_set(k_star_set.begin(), k_star_set.end());

  int k_star = sorted_k_star_set.begin()->first;
  cur_state.believe.believes[k_star] += delta_p_;
  double total = std::accumulate(cur_state.believe.believes.begin(),
                                 cur_state.believe.believes.end(), 0.0);

  for (auto& belief : cur_state.believe.believes) {
    belief /= total;
  }
  return true;
}

double LevelK::AccumulateCostWithBelief(const Vehicle& agent,
                                        const std::vector<double>& cost_set) {
  std::vector<double> costs_with_belief;

  for (size_t i = 0; i < agent.believe.believes.size(); i++) {
    costs_with_belief.emplace_back(agent.believe.believes[i] * cost_set[i]);
  }

  return std::accumulate(costs_with_belief.begin(), costs_with_belief.end(),
                         0.0);
}

void LevelK::CondidateSampling(const Vehicle& vehicle,
                               const std::string& vehicle_type,
                               std::vector<std::vector<Vehicle>>& sample_seqs) {
  Node root(vehicle);
  std::vector<Node> pending_current_nodes = {root};
  std::vector<std::vector<Node>> pending_nodes_records{};
  level_k_utils::BfsBuild(actions_, vehicle_type, vision_, delta_t_,
                          action_types_, pending_current_nodes,
                          pending_nodes_records);
  std::vector<Node> leaf_nodes = pending_nodes_records[0];
  for (Node& node : leaf_nodes) {
    std::vector<Vehicle> sample_seq;
    Node* entry = level_k_utils::DeepCopy(&node);

    while (entry->father_ != nullptr) {
      sample_seq.emplace_back(entry->data_);
      entry = level_k_utils::DeepCopy(entry->father_);
    }

    std::reverse(sample_seq.begin(), sample_seq.end());
    sample_seqs.emplace_back(sample_seq);
  }
}

// note: ego and agent can be reversed
void LevelK::TransformProcessinLevelZero(Vehicle& ego, Vehicle& agent) {
  mcts_u::Node node;
  node.state.set_current_ego_state(ego);
  node.state.set_current_agents_state(agent);
  // todo (lvyang): set ego's static traj
  node.state.set_agents_set(agent);
  node.set_visit_times(1);

  mcts_u::MctsUtils mcts(ego.max_idxs, node);
  while (!node.check_terminal()) {
    node.set_quality_value(0.0);
    mcts.MonteCarloTreeSearch(&node);
  }
  // todo (lvyang): traj forward traversal in mcts solver
  mcts.MonteCarloTreeSearch(ego.believe.trajs[0]);
}

void LevelK::TransformProcessinLevelOne(Vehicle& ego, Vehicle& agent) {
  mcts_u::Node node;
  node.state.set_current_ego_state(ego);
  node.state.set_current_agents_state(agent);
  // todo (lvyang): set agent's level 0 traj
  TransformProcessinLevelZero(agent, ego);
  node.state.set_agents_set(agent.believe.trajs[0]);
  node.set_visit_times(1);

  mcts_u::MctsUtils mcts(ego.max_idxs, node);
  while (!node.check_terminal()) {
    node.set_quality_value(0.0);
    mcts.MonteCarloTreeSearch(&node);
  }
  // todo (lvyang): traj forward traversal in mcts solver
  mcts.MonteCarloTreeSearch(ego.believe.trajs[1]);
}

void LevelK::TransformProcessinLevelTwo(Vehicle& ego, Vehicle& agent) {
  mcts_u::Node node;
  node.state.set_current_ego_state(ego);
  node.state.set_current_agents_state(agent);
  // todo (lvyang): set agent's level 1 traj
  TransformProcessinLevelOne(agent, ego);
  node.state.set_agents_set(agent.believe.trajs[1]);
  node.set_visit_times(1);

  mcts_u::MctsUtils mcts(ego.max_idxs, node);
  while (!node.check_terminal()) {
    node.set_quality_value(0.0);
    mcts.MonteCarloTreeSearch(&node);
  }
  // todo (lvyang): traj forward traversal in mcts solver
  mcts.MonteCarloTreeSearch(ego.believe.trajs[2]);
}

bool LevelK::CalWinnerTrajectory(const Vehicle& ego, const Vehicle& agent,
                                 const size_t level,
                                 std::vector<Vehicle>& winner_traj) {
  mcts_u::Node node;
  node.state.set_current_ego_state(ego);
  node.state.set_current_agents_state(agent);
  if (agent.believe.trajs[level].empty()) {
    LOG_WARN("Agent's {} level traj is empty, its id is {}", level, agent.id);
    return false;
  }
  node.state.set_agents_set(agent.believe.trajs[level]);
  node.set_visit_times(1);

  mcts_u::MctsUtils mcts(ego.max_idxs, node);
  while (!node.check_terminal()) {
    node.set_quality_value(0.0);
    mcts.MonteCarloTreeSearch(&node);
  }
  // todo (lvyang): traj forward traversal in mcts solver
  mcts.MonteCarloTreeSearch(winner_traj);
  return true;
}

bool LevelK::CalWinnerTrajectory(const Vehicle& ego,
                                 const std::vector<Vehicle>& agents,
                                 std::vector<Vehicle>& winner_traj) {
  if (agents.empty()) {
    LOG_INFO("There aren't agents!");
    return false;
  }
  mcts_u::Node node;
  node.state.set_current_ego_state(ego);
  node.state.set_current_agents_state(agents);
  std::vector<std::vector<Vehicle>> agents_set{};
  for (const auto& iter : agents) {
    if (iter.traj.empty()) {
      LOG_INFO("Obs's traj is empty, its id is {}", iter.id);
      return false;
    }
  }
  for (size_t i = 0; i < agents[0].traj.size(); ++i) {
    std::vector<Vehicle> frame_agents{};
    for (const auto& iter : agents) {
      frame_agents.emplace_back(iter.traj[i]);
    }
    agents_set.emplace_back(frame_agents);
  }
  node.state.set_agents_set(agents_set);
  node.set_visit_times(1);

  mcts_u::MctsUtils mcts(ego.max_idxs, node);
  while (!node.check_terminal()) {
    node.set_quality_value(0.0);
    mcts.MonteCarloTreeSearch(&node);
  }
  mcts.MonteCarloTreeSearch(winner_traj);
  return true;
}

bool LevelK::MakeEgoCopingStrategy(Vehicle& ego, Vehicle& agent,
                                   std::vector<Vehicle>& winner_stategy,
                                   Trajectory& traj) {
  // temporarily: calc every agent's respond traj, winner traj and solved
  // future: calc every agent's expectations for ego, include in global reward
  std::map<int, double> all_coping_stategies;

  TransformProcessinLevelZero(ego, agent);
  TransformProcessinLevelOne(ego, agent);
  TransformProcessinLevelTwo(ego, agent);

  std::vector<Vehicle> level_zero_ego_trajectory{};
  if (CalWinnerTrajectory(ego, agent, 0, level_zero_ego_trajectory)) {
    return false;
  }

  std::vector<Vehicle> level_one_ego_trajectory{};
  if (CalWinnerTrajectory(ego, agent, 1, level_one_ego_trajectory)) {
    return false;
  }

  std::vector<Vehicle> level_two_ego_trajectory{};
  if (CalWinnerTrajectory(ego, agent, 2, level_two_ego_trajectory)) {
    return false;
  }

  // todo (lvyang): To be cross validated
  level_k_context::CrossCheckData ccd;

  // zero
  for (int index = 0; index < level_zero_ego_trajectory.size(); index++) {
    Vehicle pt = level_zero_ego_trajectory[index];
    ccd.costs_zero_vs["zero"].emplace_back(
        CostFunction(pt, agent.believe.trajs[0][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_zero_vs["one"].emplace_back(
        CostFunction(pt, agent.believe.trajs[1][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_zero_vs["two"].emplace_back(
        CostFunction(pt, agent.believe.trajs[2][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
  }
  for (int i = 0; i < ccd.costs_zero_vs["zero"].size(); i++) {
    ccd.accumulate_cost_zero_vs["zero"] +=
        AccumulateCost(ccd.costs_zero_vs["zero"][i], i);
    ccd.accumulate_cost_zero_vs["one"] +=
        AccumulateCost(ccd.costs_zero_vs["one"][i], i);
    ccd.accumulate_cost_zero_vs["two"] +=
        AccumulateCost(ccd.costs_zero_vs["two"][i], i);
  }
  std::vector<double> cost_set = {ccd.accumulate_cost_zero_vs["zero"],
                                  ccd.accumulate_cost_zero_vs["one"],
                                  ccd.accumulate_cost_zero_vs["two"]};
  ccd.accumulate_level_cost["zero"] = AccumulateCostWithBelief(ego, cost_set);

  // one
  for (int index = 0; index < level_one_ego_trajectory.size(); index++) {
    Vehicle pt = level_one_ego_trajectory[index];
    ccd.costs_one_vs["zero"].emplace_back(
        CostFunction(pt, agent.believe.trajs[0][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_one_vs["one"].emplace_back(
        CostFunction(pt, agent.believe.trajs[1][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_one_vs["two"].emplace_back(
        CostFunction(pt, agent.believe.trajs[2][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
  }
  for (int i = 0; i < ccd.costs_one_vs["zero"].size(); i++) {
    ccd.accumulate_cost_one_vs["zero"] +=
        AccumulateCost(ccd.costs_one_vs["zero"][i], i);
    ccd.accumulate_cost_one_vs["one"] +=
        AccumulateCost(ccd.costs_one_vs["one"][i], i);
    ccd.accumulate_cost_one_vs["two"] +=
        AccumulateCost(ccd.costs_one_vs["two"][i], i);
  }
  cost_set.clear();
  cost_set = {ccd.accumulate_cost_one_vs["zero"],
              ccd.accumulate_cost_one_vs["one"],
              ccd.accumulate_cost_one_vs["two"]};
  ccd.accumulate_level_cost["one"] = AccumulateCostWithBelief(ego, cost_set);

  // two
  for (int index = 0; index < level_two_ego_trajectory.size(); index++) {
    Vehicle pt = level_two_ego_trajectory[index];
    ccd.costs_two_vs["zero"].emplace_back(
        CostFunction(pt, agent.believe.trajs[0][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_two_vs["one"].emplace_back(
        CostFunction(pt, agent.believe.trajs[1][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
    ccd.costs_two_vs["two"].emplace_back(
        CostFunction(pt, agent.believe.trajs[2][index], false, false,
                     ego.goal_pt, ego.ref_pts, Vec2d(pt.state.x, pt.state.y)));
  }
  for (int i = 0; i < ccd.costs_two_vs["zero"].size(); i++) {
    ccd.accumulate_cost_two_vs["zero"] +=
        AccumulateCost(ccd.costs_two_vs["zero"][i], i);
    ccd.accumulate_cost_two_vs["one"] +=
        AccumulateCost(ccd.costs_two_vs["one"][i], i);
    ccd.accumulate_cost_two_vs["two"] +=
        AccumulateCost(ccd.costs_two_vs["two"][i], i);
  }

  cost_set.clear();
  cost_set = {ccd.accumulate_cost_two_vs["zero"],
              ccd.accumulate_cost_two_vs["one"],
              ccd.accumulate_cost_two_vs["two"]};
  ccd.accumulate_level_cost["two"] = AccumulateCostWithBelief(ego, cost_set);

  all_coping_stategies[0] = ccd.accumulate_level_cost["zero"];
  all_coping_stategies[1] = ccd.accumulate_level_cost["one"];
  all_coping_stategies[2] = ccd.accumulate_level_cost["two"];

  std::map<int, double> sorted_all_coping_stategies(
      all_coping_stategies.begin(), all_coping_stategies.end());
  std::map<int, double>::iterator it = sorted_all_coping_stategies.begin();
  int winner_id = it->first;

  if (winner_id == 0) {
    winner_stategy = level_zero_ego_trajectory;
  } else if (winner_id == 1) {
    winner_stategy = level_one_ego_trajectory;
  } else if (winner_id == 2) {
    winner_stategy = level_two_ego_trajectory;
  }
  TransVehSetToTraj(winner_stategy, traj);
}

void LevelK::TransVehSetToTraj(const std::vector<Vehicle>& veh_seq,
                               Trajectory& traj) {
  if (veh_seq.empty()) {
    return;
  }
  for (size_t i = 0; i < veh_seq.size(); ++i) {
    TrajPoint pt;
    pt.Reset(veh_seq[i].state, i);
    traj.pts.emplace_back(pt);
  }
}
}  // namespace planning
}  // namespace neodrive