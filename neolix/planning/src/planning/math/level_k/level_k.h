#pragma once
#include <algorithm>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "context.h"
#include "level_k_utils.h"
#include "mcts.h"
#include "src/planning/common/math/vec2d.h"

namespace neodrive {
namespace planning {
class LevelK {
 public:
  using Vehicle = level_k_context::Vehicle;
  using Actions = level_k_context::Actions;
  using Weights = level_k_context::Weights;
  using Believes = level_k_context::Believes;
  using Node = level_k_context::Node;
  using Trajectory = level_k_context::Trajectory;
  using TrajPoint = level_k_context::TrajPoint;
  using State = level_k_context::State;

  LevelK(const Vehicle& ego, const std::vector<Vehicle>& agents,
         const std::vector<Vehicle>& last_agents)
      : ego_(ego), agents_(agents), last_states_(last_agents){};

  bool CalcGlobalOptimalTrajectory();

  bool CalcLocalOptimalTrajectory();

  bool UpdateBelief(std::vector<std::vector<Vehicle>>& last_planning,
                    Vehicle& cur_state);

  Believes get_belief() const { return believes_; }

 private:
  double CostFunction(const Vehicle& ego, const Vehicle& agent,
                      const bool& off_road_flag, const bool& retrograde_flag,
                      const Vec2d& goal_pt,
                      const std::vector<Vec2d>& refline_pts,
                      const Vec2d& search_pt);

  double CostFunction(const std::vector<Vehicle>& traj_ego,
                      const std::vector<Vehicle>& traj_agent,
                      const Vec2d& goal_pt, const double goal_s,
                      const Vec2d& search_pt);

  double AccumulateCost(const double cost, const int n);

  double AccumulateCostWithBelief(const Vehicle& agent,
                                  const std::vector<double>& cost_set);

  void CondidateSampling(const Vehicle& vehicle,
                         const std::string& vehicle_type,
                         std::vector<std::vector<Vehicle>>& sample_seqs);

  void TransformProcessinLevelZero(Vehicle& ego, Vehicle& agent);

  void TransformProcessinLevelOne(Vehicle& ego, Vehicle& agent);

  void TransformProcessinLevelTwo(Vehicle& ego, Vehicle& agent);

  bool CalWinnerTrajectory(const Vehicle& ego, const Vehicle& agent,
                           const size_t level,
                           std::vector<Vehicle>& winner_traj);

  bool CalWinnerTrajectory(const Vehicle& ego,
                           const std::vector<Vehicle>& agents,
                           std::vector<Vehicle>& winner_traj);

  bool MakeEgoCopingStrategy(Vehicle& ego, Vehicle& agent,
                             std::vector<Vehicle>& winner_stategy,
                             Trajectory& traj);

  void TransVehSetToTraj(const std::vector<Vehicle>& veh_seq, Trajectory& traj);

  std::vector<std::vector<Vehicle>> get_ego_all_trajs() const {
    return ego_all_trajectories_;
  }

  Trajectory get_ego_best_traj() const { return ego_.goal_traj; }

  std::vector<std::vector<Vehicle>> get_agent_all_trajs() const {
    return agent_all_trajectories_;
  }

  Trajectory get_other_traj_level0(const level_k_context::Vehicle& v) {
    Trajectory traj;
    TransVehSetToTraj(v.believe.trajs[0], traj);
    return traj;
  }

  Trajectory get_other_traj_level1(const level_k_context::Vehicle& v) {
    Trajectory traj;
    TransVehSetToTraj(v.believe.trajs[1], traj);
    return traj;
  }

  Trajectory get_other_traj_level2(const level_k_context::Vehicle& v) {
    Trajectory traj;
    TransVehSetToTraj(v.believe.trajs[2], traj);
    return traj;
  }

  void set_belief(const Believes& belief) { believes_ = belief; }

 private:
  Weights weights_;
  Believes believes_;
  Actions actions_;
  double delta_t_{0.1};
  double vision_{1.8};
  double lambda_{0.9};
  double delta_p_{0.5};
  double safedist_lat_{1.0};
  double safedist_lon_{2.0};
  Vehicle ego_;
  std::vector<Vehicle> agents_;

  std::vector<Vehicle> last_states_;
  std::vector<std::vector<Vehicle>> ego_all_trajectories_{};
  std::vector<std::vector<Vehicle>> agent_all_trajectories_{};
  // std::vector<Vehicle> car_traj_level_0_{};
  // std::vector<Vehicle> car_traj_level_1_{};
  // std::vector<Vehicle> car_traj_level_2_{};
  std::vector<std::string> action_types_ = {"Acc", "Dec", "Maintain",
                                            "Turnleft", "Turnright"};
};
}  // namespace planning
}  // namespace neodrive