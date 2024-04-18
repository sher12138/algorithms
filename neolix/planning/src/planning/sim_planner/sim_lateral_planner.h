#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "calculations.h"
#include "cfg.h"
#include "dcp_tree.h"
#include "onlane_forward_simulation.h"
#include "sim_map.h"
#include "sim_task.h"
#include "state.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

const double kInf = 1e20;
const double kEPS = 1e-6;

class SimLateralPlanner {
 public:
  using Cfg = neodrive::planning::sim_planner::Cfg;
  using DcpTree = neodrive::planning::sim_planner::DcpTree;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  enum class LatSimMode {
    kAlwaysLaneKeep = 0,
    kKeepThenChange,
    kAlwaysLaneChange,
    kChangeThenCancel
  };

  struct ForwardSimEgoAgent {
    OnLaneForwardSimulation::Param sim_param;
    LatSimMode seq_lat_mode;
    LateralBehavior seq_lat_behavior;
    bool is_cancel_behavior;
    double operation_at_seconds{0.0};
    LongitudinalBehavior lon_behavior{LongitudinalBehavior::kMaintain};
    LateralBehavior lat_behavior{LateralBehavior::kUndefined};
    int current_lane;
    int target_lane;
    std::unordered_map<std::string, neodrive::planning::Boundary> obs_boundary;
    double s_bound;  // Road boundary modification (virtual obstacle, to be
                     // defined)
    Vehicle vehicle;
  };

  struct ForwardSimAgent {
    int id = -1;
    Vehicle vehicle;
  };

  struct ForwardSimAgentSet {
    std::unordered_map<int, ForwardSimAgent> forward_sim_agents;
  };

  struct EfficiencyCost {
    double ego_to_desired_vel = 0.0;
    double leading_to_desired_vel = 0.0;
    double ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  struct NavigationCost {
    double lane_change_preference = 0.0;
    double ave() const { return lane_change_preference; }
  };

  struct OffsetDrivingCost {
    double center_driving_preference = 0.0;
    double ave() const { return center_driving_preference; }
  };

  struct CostStructure {
    int valid_sample_index_ub;
    // * efficiency
    EfficiencyCost efficiency;
    // * safety
    // SafetyCost safety;
    // * navigation
    NavigationCost navigation;
    // * Centered driving
    OffsetDrivingCost center_driving;
    int lane_id = 999;
    // test
    bool has_leading_obs_center = false;
    int potential_left_id = 999, potential_center_id = 999,
        potential_right_id = 999;
    int cur_id = 999, target_id = 999;
    double weight = 1.0;
    double ave() const {
      return (efficiency.ave() + navigation.ave() + center_driving.ave()) *
             weight;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << ")"
         << ", navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  bool init();

  bool runOnce();

  void setVehicleState(const State& state);

  void setDesiredVelocity(const double desired_vel);

  void setLaneChangeInfo(const LaneChangeInfo& lc_info);

  bool runEudm(const std::vector<std::vector<DcpAction>>& action_script);

  std::vector<DcpAction> winner_action_seq() const {
    return winner_action_seq_;
  }

  int winner_id() const;

  double time_cost() const;

  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.emplace_back(false);
      } else {
        ret.emplace_back(true);
      }
    }
    return ret;
  }

  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.emplace_back(false);
      } else {
        ret.emplace_back(true);
      }
    }
    return ret;
  }

  std::vector<std::string> sim_info() const { return sim_info_; }
  std::vector<double> final_cost() const { return final_cost_; }
  std::vector<std::vector<CostStructure>> progress_cost() const {
    return progress_cost_;
  }
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }
  State plan_state() { return ego_vehicle_.state(); }
  std::vector<std::vector<DcpAction>> action_script() {
    auto action_script = dcp_tree_ptr_->action_script();
    action_script.emplace_back(std::vector<DcpAction>(
        5, DcpAction(DcpLonAction(1), DcpLatAction(1), 1.0)));
    action_script.emplace_back(std::vector<DcpAction>(
        5, DcpAction(DcpLonAction(1), DcpLatAction(2), 1.0)));
    return action_script;
  }

  const Cfg& cfg() const { return cfg_; }

  void updateDcpTree(const DcpAction& ongoing_action);

  bool classifyActionSeq(const std::vector<DcpAction>& action_seq,
                         double* operation_at_seconds,
                         LateralBehavior* lat_behavior,
                         bool* is_cancel_operation) const;

  std::vector<std::vector<Vehicle>> forward_trajs() const {
    return forward_trajs_;
  }

  neodrive::planning::Boundary last_leading_boundary() const {
    return last_leading_boundary_;
  }

  void set_last_leading_boundary(const neodrive::planning::Boundary& boundary) {
    last_leading_boundary_.set_start_s(boundary.start_s());
    last_leading_boundary_.set_end_s(boundary.end_s());
    last_leading_boundary_.set_start_l(boundary.start_l());
    last_leading_boundary_.set_end_l(boundary.end_l());
  }

  bool pruningTwoLanes(const double cur_l, const int center_id,
                       const int left_id, const int right_id,
                       const std::vector<DcpAction>& action_seq);

  int potential_left_id() { return potential_left_id_; }
  int potential_center_id() { return potential_center_id_; }
  int potential_right_id() { return potential_right_id_; }

  //  private:
 public:
  bool getEgoSimParam(const Cfg& cfg,
                      OnLaneForwardSimulation::Param* sim_param);

  bool translateDcpActionToLonLatBehavior(const DcpAction& action,
                                          LateralBehavior* lat,
                                          LongitudinalBehavior* lon) const;

  // * simulation control loop
  bool simulateActionSequence(
      const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
      const int seq_id, const neodrive::planning::Boundary& center_boundary,
      const double cur_s);

  bool simulateScenario(
      const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
      const int seq_id, const int sub_seq_id,
      const neodrive::planning::Boundary& center_boundary, const double cur_s,
      std::vector<int>* sub_sim_res, std::vector<int>* sub_risky_res,
      std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructure>>* sub_progress_cost,
      std::vector<std::vector<Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>*
          sub_forward_lon_behaviors);

  bool simulateSingleAction(const DcpAction& action,
                            const ForwardSimEgoAgent& ego_fsagent_this_layer,
                            std::vector<Vehicle>* ego_traj);

  // * evaluation functions
  bool costFunction(const DcpAction& action,
                    const ForwardSimEgoAgent& ego_fsagent, const bool verbose,
                    const int current_lane_id, CostStructure* cost);

  bool strictSafetyCheck(const std::vector<Vehicle>& ego_traj, bool* is_safe);
  bool strictSingleSafetyCheck(const Vehicle& vehicle, bool* is_safe) const;
  bool evaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& progress_cost,
      const std::vector<DcpAction>& action_seq, double* score);

  bool getLookAheadDistOnLC(
      const ForwardSimEgoAgent& ego,
      const neodrive::planning::Boundary& leading_boundary, const double alpha,
      const double delta, double* look_ahead_dist) const;
  bool checkOverStepBound(const State& state, const double threshold,
                          bool* res);

  bool evaluateMultiThreadSimResults(
      const std::vector<std::vector<DcpAction>>& action_script, int* winner_id,
      double* winner_cost);

  // * simulation util functions
  bool updateSimSetupForScenario(const std::vector<DcpAction>& action_seq,
                                 ForwardSimEgoAgent* ego_fsagent) const;

  bool updateSimSetupForLayer(const DcpAction& action,
                              ForwardSimEgoAgent* ego_fsagent) const;

  bool checkIfLateralActionFinished(const State& cur_state,
                                    const int target_lane_id,
                                    const int current_lane_id,
                                    const LateralBehavior& lat_behavior) const;

  bool updateLateralActionSequence(const int cur_idx,
                                   std::vector<DcpAction>* action_seq) const;

  bool prepareMultiThreadContainers(const int n_sequence);

  bool getSimTimeSteps(const DcpAction& action,
                       std::vector<double>* dt_steps) const;

  bool egoAgentForwardSim(ForwardSimEgoAgent& ego_fsagent,
                          const double sim_time_step, State* state_out) const;

  // bool SurroundingAgentForwardSim(
  //     const ForwardSimAgent& fsagent,
  //     const VehicleSet& all_sim_vehicles,
  //     const double& sim_time_step, State* state_out) const;
 private:
  // * action
  DcpTree* dcp_tree_ptr_;
  // * setup
  Cfg cfg_;
  LaneChangeInfo lc_info_;
  double desired_velocity_{5.0};
  double sim_time_total_ = 0.0;
  std::set<int> pre_deleted_seq_ids_;
  int ego_lane_id_{-999};

  OnLaneForwardSimulation::Param ego_sim_param_;
  OnLaneForwardSimulation::Param agent_sim_param_;

  double time_stamp_;
  int ego_id_;
  Vehicle ego_vehicle_;

  // * result
  int winner_id_ = 0;
  double winner_score_ = 0.0;
  std::vector<DcpAction> winner_action_seq_;
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<double> final_cost_;
  std::vector<std::vector<CostStructure>> progress_cost_;
  std::vector<std::vector<Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  neodrive::planning::Boundary last_leading_boundary_{
      .s_s = 0.0, .e_s = 0.0, .s_l = 0.0, .e_l = 0.0};
  double time_cost_ = 0.0;

  // dynamic lane id
  int potential_left_id_ = 0, potential_right_id_ = 0, potential_center_id_ = 0;
};
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
