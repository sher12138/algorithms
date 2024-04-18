#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <unsupported/Eigen/MatrixFunctions>

#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/task/deciders/speed_constrained_iter_lqr_planner/speed_cilqr_model.h"
#include "src/planning/task/deciders/speed_constrained_iter_lqr_planner/speed_cilqr_obs.h"
namespace neodrive {
namespace planning {
struct ObsBoundInfo {
  std::vector<ObsDecisionBound> obs_low_bound{};

  std::vector<ObsDecisionBound> obs_up_bound{};
  void SetBound(std::vector<ObsDecisionBound> a,
                std::vector<ObsDecisionBound> b) {
    obs_low_bound = a;
    obs_up_bound = b;
  }
  void Reset() {
    obs_low_bound.clear();
    obs_up_bound.clear();
  }
};

struct BackwardPerturbationPara {
  std::vector<CilqrModel::l_x> l_x{};
  std::vector<CilqrModel::l_xx> l_xx{};
  std::vector<CilqrModel::l_u> l_u{};
  std::vector<CilqrModel::l_uu> l_uu{};
  std::vector<CilqrModel::l_ux> l_ux{};
};

struct BackwardPerturbationParaControl {
  std::vector<CilqrModel::l_u> l_u{};
  std::vector<CilqrModel::l_uu> l_uu{};
};

struct BackwardPerturbationParaState {
  std::vector<CilqrModel::l_x> l_x{};
  std::vector<CilqrModel::l_xx> l_xx{};
};

struct OptResult {
  std::vector<CilqrModel::CilqrState> state_seq{};
  std::vector<CilqrModel::CilqrControl> control_seq{};
  void Reset() {
    state_seq.clear();
    control_seq.clear();
  }
};

struct BarrierFunctionFeedBack {
  double exponential_cost;
  Eigen::MatrixXd exponential_cost_d;
  Eigen::MatrixXd exponential_cost_dd;
};
struct RefSpeedPlan {
  std::vector<STGoalSInfo> goal_s_{};
  std::vector<STGoalVInfo> goal_v_{};
  std::vector<STGoalAInfo> goal_a_{};
  void SetBound(std::vector<STGoalSInfo> a, std::vector<STGoalVInfo> b,
                std::vector<STGoalAInfo> c) {
    goal_s_ = a;
    goal_v_ = b;
    goal_a_ = c;
  }
  void Reset() {
    goal_s_.clear();
    goal_v_.clear();
    goal_a_.clear();
  }
};
class RunStepCilqr {
 public:
  ~RunStepCilqr() = default;
  RunStepCilqr() = default;
  RunStepCilqr(
      const CilqrModel::CilqrState& ego_state,
      const std::vector<std::shared_ptr<SpeedCilqrObsProcess>>& sorted_obs_list,
      const RefSpeedPlan& ref_speed_plan, const size_t& steps);
  std::vector<CilqrModel::CilqrState> Optimize();

  std::vector<CilqrModel::CilqrState> GetOptimalControlSeq();

  std::vector<CilqrModel::CilqrState> ClampErrorData(
      std::vector<CilqrModel::CilqrState>& ego_deduction_data);

  bool CheckIllness(std::vector<CilqrModel::CilqrState>& ego_deduction_data);

  std::vector<CilqrModel::CilqrState> GetRoughStateSeq(
      const CilqrModel::CilqrState& ego_state,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq);

  std::pair<std::vector<CilqrModel::K>, std::vector<CilqrModel::k>>
  BackwardPass(const std::vector<CilqrModel::CilqrState>& rough_state_seq,
               const std::vector<CilqrModel::CilqrControl>& rough_control_seq,
               const double& lambda);

  std::pair<std::vector<double>, std::vector<CilqrModel::X_state>> ForwardPass(
      const std::vector<CilqrModel::CilqrState>& rough_state_seq,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq,
      const std::pair<std::vector<CilqrModel::K>, std::vector<CilqrModel::k>>&
          best_control_rate);
  void reset();

  BackwardPerturbationPara GetCostDerivatives(
      const std::vector<CilqrModel::CilqrState>& rough_state_seq,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq);

  BackwardPerturbationParaControl GetControlCostDerivatives(
      const std::vector<CilqrModel::CilqrState>& rough_state_seq,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq);

  BackwardPerturbationParaState GetStateCostDerivatives(
      const std::vector<CilqrModel::CilqrState>& rough_state_seq,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq);

  BarrierFunctionFeedBack BarrireFunction(double q1, double q2, double error,
                                          Eigen::MatrixXd error_dot);

  CilqrModel::X_state ForwardSimulation(const CilqrModel::X_state& x_state_old,
                                        const double& control_jerk);

  void UpdateData(
      std::vector<CilqrModel::CilqrState>& rough_state_seq,
      std::vector<CilqrModel::CilqrControl>& rough_control_seq,
      const std::pair<std::vector<double>, std::vector<CilqrModel::X_state>>&
          forward_result);

  double GetTotalCost(
      const std::vector<CilqrModel::CilqrState>& rough_state_seq,
      const std::vector<CilqrModel::CilqrControl>& rough_control_seq);

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(bool, is_illness);

 private:
  CilqrModel::CilqrState ego_state_;
  static std::shared_ptr<SpeedCilqrModel> cilqr_model_;
  std::vector<std::shared_ptr<SpeedCilqrObsProcess>> obs_list_;
  RefSpeedPlan ref_speed_plan_;
  double delta_t_;
  size_t steps_;
  size_t max_iters_;
  constexpr static int kNoise = 5;
  bool is_illness_{false};
};

}  // namespace planning
}  // namespace neodrive