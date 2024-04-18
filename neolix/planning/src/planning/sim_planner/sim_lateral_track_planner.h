#pragma once
#include "sim_lateral_planner.h"
namespace neodrive {
namespace planning {
namespace sim_planner {
class SimLateralTrackPlanner : public SimLateralPlanner {
 public:
  using DcpTree = neodrive::planning::sim_planner::DcpTree;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  struct GoalCost {
    double goal_dis_cost = 0.0;
    double ave() const { return goal_dis_cost; }
  };
  struct SmoothCost {
    double accumulate_k_cost = 0.0;
    double ave() const { return accumulate_k_cost; }
  };
  struct Efficiency {
    double ego_to_desired_vel = 0;
    double ave() const { return ego_to_desired_vel; }
  };
  struct CostStructureRP {
    GoalCost goal_cost;
    SmoothCost smooth_cost;
    Efficiency efficiency_;
    double weight = 1.0;
    double ave() const {
      return (goal_cost.ave() + smooth_cost.ave() + efficiency_.ave()) * weight;
    }
    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructureRP& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(goal cost: " << cost.goal_cost.goal_dis_cost
         << "), smooth cost: " << cost.smooth_cost.accumulate_k_cost << ")";
      return os;
    }
  };
  void set_goal_pt(const Vec2d& pt) { goal_pt_ = pt; }

  void set_goal_sl_pt(const SimMapPoint& sl_pt) { goal_sl_pt_ = sl_pt; }

  Vec2d goal_pt() const { return goal_pt_; }

  std::vector<std::vector<CostStructureRP>> progress_cost() const {
    return progress_cost_;
  }

  std::vector<std::vector<Vehicle>> forward_trajs() const {
    return forward_trajs_;
  }

  void set_goal_pts(const std::vector<Vec2d>& goal_pts) {
    goal_pts_ = goal_pts;
  }

  void set_goal_sl_pts(const std::vector<SimMapPoint>& goal_sl_pts) {
    goal_sl_pts_ = goal_sl_pts;
  }

  void setVehicleState(const State& state) { ego_vehicle_.setState(state); }

  SimMapPoint goal_sl_pt() const { return goal_sl_pt_; }
  bool runOnce(const std::vector<SimMapPoint>& goal_sl_pts,
               const std::unordered_map<std::string, int>& all_lanes_id);

  bool runEudm(const std::vector<std::vector<DcpAction>>& action_seq);

  bool simulateActionSequence(const Vehicle& ego_vehicle,
                              const std::vector<DcpAction>& action_seq,
                              const int seq_id);

  bool checkOverStepBound(const State& state, const double threshold,
                          bool* res);

  bool simulateScenario(
      const Vehicle& ego_vehicle, const std::vector<DcpAction>& action_seq,
      const int seq_id, const int sub_seq_id, std::vector<int>* sub_sim_res,
      std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
      std::vector<std::vector<CostStructureRP>>* sub_progress_cost,
      std::vector<std::vector<Vehicle>>* sub_forward_trajs,
      std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
      std::vector<std::vector<LongitudinalBehavior>>*
          sub_forward_lon_behaviors);

  bool updateSimSetupForLayer(const DcpAction& action,
                              ForwardSimEgoAgent* ego_fsagent) const;

  bool simulateSingleAction(const DcpAction& action,
                            const ForwardSimEgoAgent& ego_fsagent_this_layer,
                            std::vector<Vehicle>* ego_traj);

  bool egoAgentForwardSim(ForwardSimEgoAgent& ego_fsagent,
                          const double sim_time_step, State* state_out) const;

  bool costFunction(const DcpAction& action,
                    const ForwardSimEgoAgent& ego_fsagent, const bool verbose,
                    const std::vector<Vehicle>& traj_set,
                    CostStructureRP* cost);

  bool prepareMultiThreadContainers(const int n_sequence);

  bool evaluateMultiThreadSimResults(
      const std::vector<std::vector<DcpAction>>& action_seq, int* winner_id,
      double* winner_cost);

  bool evaluateSinglePolicyTrajs(
      const std::vector<CostStructureRP>& progress_cost, double* score);

  int winner_id() const;

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

  State plan_state() { return ego_vehicle_.state(); }

  std::vector<std::string> sim_info() const { return sim_info_; }

  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }

  // bool ResetGoalPt(const double s, std::vector<Vec2d>& goal_pts) {
  //   if (goal_pts.empty()) {
  //     return false;
  //   }
  //   if (s > goal_sl_pt_.s) {
  //     goal_pt_ = goal_pts.front();
  //     SimMap::Instance()->getNearestPoint(goal_pt, &goal_sl_pt_);
  //     goal_pts.erase(goal_pts.begin());
  //   }
  //   return true;
  // }

 private:
  Vec2d goal_pt_;
  SimMapPoint goal_sl_pt_;
  std::vector<Vec2d> goal_pts_;
  std::vector<SimMapPoint> goal_sl_pts_;
  neodrive::planning::sim_planner::Cfg cfg_;
  double desired_velocity_{5.0};
  int ego_lane_id_{-999};

  double time_stamp_;
  int ego_id_;
  Vehicle ego_vehicle_;
  std::set<int> pre_deleted_seq_ids_;
  DcpTree* dcp_tree_ptr_;

  int winner_id_ = 0;
  double winner_score_ = 0.0;
  std::vector<DcpAction> winner_action_seq_;
  std::vector<int> sim_res_;
  std::vector<int> risky_res_;
  std::vector<std::string> sim_info_;
  std::vector<double> final_cost_;
  std::vector<std::vector<CostStructureRP>> progress_cost_;
  std::vector<std::vector<Vehicle>> forward_trajs_;
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_;
  double time_cost_ = 0.0;
  int potential_left_id_ = 0, potential_center_id_ = 0, potential_right_id_ = 0;
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive