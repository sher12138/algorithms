#pragma once

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/math/hpipm_solver/third_order_spline_speed_hpipm_solver.h"
#include "third_order_spline_speed_model.h"

namespace neodrive {
namespace planning {

class ThirdOrderSplineSpeedMPC {
 public:
  ThirdOrderSplineSpeedMPC() = delete;
  ThirdOrderSplineSpeedMPC(const double delta_t, const int n, const int nx,
                           const int nu, const int npc, const int ns,
                           const ThirdOrderSplineSpeed::State& init_state,
                           const ThirdOrderSplineSpeed::Control& init_control,
                           const double speed_limit,
                           const std::vector<double>& upper_s_bounds,
                           const std::vector<double>& lower_s_bounds,
                           const std::vector<double>& upper_v_bounds,
                           const std::vector<double>& lower_v_bounds,
                           const std::vector<double>& upper_a_bounds,
                           const std::vector<double>& lower_a_bounds,
                           const std::vector<double>& upper_jerk_bounds,
                           const std::vector<double>& lower_jerk_bounds,
                           const std::vector<STGoalSInfo>& goal_upper_s,
                           const std::vector<STGoalVInfo>& goal_upper_v,
                           const std::vector<STGoalSInfo>& goal_lower_s,
                           const std::vector<STGoalVInfo>& goal_lower_v,
                           const std::vector<STGoalAInfo>& goal_v);

  ~ThirdOrderSplineSpeedMPC();

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<ThirdOrderSplineSpeed::OptVariables>, optimal_solution)

 private:
  bool RunMPC();

  bool SetMPCProblem();

  bool SetStage(const ThirdOrderSplineSpeed::State& x_k,
                const ThirdOrderSplineSpeed::Control& u_k, const int step,
                const double weight_s, const double weight_v,
                const double weight_a);
  void CalWeight(double* s_weight, double* v_weight, double* a_weight);

 private:
  double delta_t_{0.1};
  int n_{};
  int nx_{};
  int nu_{};
  int npc_{};
  int ns_{};
  ThirdOrderSplineSpeed::State init_state_{};
  ThirdOrderSplineSpeed::Control init_control_{};
  double speed_limit_{};
  std::vector<double> upper_s_bounds_{};
  std::vector<double> lower_s_bounds_{};
  std::vector<double> upper_v_bounds_{};
  std::vector<double> lower_v_bounds_{};
  std::vector<double> upper_a_bounds_{};
  std::vector<double> lower_a_bounds_{};
  std::vector<double> upper_jerk_bounds_{};
  std::vector<double> lower_jerk_bounds_{};
  std::vector<STGoalSInfo> goal_upper_s_{};
  std::vector<STGoalVInfo> goal_upper_v_{};
  std::vector<STGoalSInfo> goal_lower_s_{};
  std::vector<STGoalVInfo> goal_lower_v_{};
  std::vector<STGoalAInfo> goal_a_{};

  std::shared_ptr<ThirdOrderSplineSpeedModel> model_{};
  std::vector<ThirdOrderSplineSpeed::Stage> stages_{};
  std::vector<ThirdOrderSplineSpeed::OptVariables> optimal_solution_{};
  std::shared_ptr<ThirdOrderSplineSpeedHpipmSolver> mpc_solver_{};
};
}  // namespace planning
}  // namespace neodrive