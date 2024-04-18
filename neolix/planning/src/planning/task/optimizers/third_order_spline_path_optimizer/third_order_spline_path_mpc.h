#pragma once

#include "src/planning/common/planning_macros.h"
#include "src/planning/math/hpipm_solver/third_order_spline_path_hpipm_solver.h"
#include "third_order_spline_path_model.h"

namespace neodrive {
namespace planning {

class ThirdOrderSplinePathMPC {
 public:
  ThirdOrderSplinePathMPC() = delete;
  ThirdOrderSplinePathMPC(const double car_length, const double cur_speed,
                          const std::vector<double>& delta_s_vector,
                          const ThirdOrderSplinePath::State& init_state,
                          const ThirdOrderSplinePath::Control& init_control,
                          const std::vector<double>& upper_l_0_bounds,
                          const std::vector<double>& lower_l_0_bounds,
                          const std::vector<double>& upper_l_1_bounds,
                          const std::vector<double>& lower_l_1_bounds,
                          const std::vector<double>& upper_l_2_bounds,
                          const std::vector<double>& lower_l_2_bounds,
                          const std::vector<double>& upper_dl_0_bounds,
                          const std::vector<double>& lower_dl_0_bounds,
                          const std::vector<double>& upper_ddl_0_bounds,
                          const std::vector<double>& lower_ddl_0_bounds,
                          const std::vector<double>& upper_dddl_0_bounds,
                          const std::vector<double>& lower_dddl_0_bounds,
                          const std::vector<double>& goal_l0,
                          const std::vector<double>& goal_l1,
                          const std::vector<double>& goal_l2);

  ~ThirdOrderSplinePathMPC();

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<ThirdOrderSplinePath::OptVariables>, optimal_solution)

 private:
  bool RunMPC();

  bool SetMPCProblem();

  bool SetStage(const ThirdOrderSplinePath::State& x_k,
                const ThirdOrderSplinePath::Control& u_k, const int step);

  double CalcGoalVarianceGain(const std::vector<double>& goal_path,
                              const std::vector<double>& delta_s_vector,
                              const double cur_speed);

 private:
  double car_length_{};
  std::vector<double> delta_s_vector_{};
  ThirdOrderSplinePath::State init_state_{};
  ThirdOrderSplinePath::Control init_control_{};
  std::vector<double> upper_l_0_bounds_{};
  std::vector<double> lower_l_0_bounds_{};
  std::vector<double> upper_l_1_bounds_{};
  std::vector<double> lower_l_1_bounds_{};
  std::vector<double> upper_l_2_bounds_{};
  std::vector<double> lower_l_2_bounds_{};
  std::vector<double> upper_dl_0_bounds_{};
  std::vector<double> lower_dl_0_bounds_{};
  std::vector<double> upper_ddl_0_bounds_{};
  std::vector<double> lower_ddl_0_bounds_{};
  std::vector<double> upper_dddl_0_bounds_{};
  std::vector<double> lower_dddl_0_bounds_{};
  std::vector<double> goal_l0_{};
  std::vector<double> goal_l1_{};
  std::vector<double> goal_l2_{};

  int n_{};
  int nx_{};
  int nu_{};
  int npc_{};
  int ns_{};

  std::shared_ptr<ThirdOrderSplinePathModel> path_model_{};
  std::vector<ThirdOrderSplinePath::Stage> stages_{};
  std::shared_ptr<ThirdOrderSplinePathHpipmSolver> mpc_solver_{};
  std::vector<ThirdOrderSplinePath::OptVariables> optimal_solution_{};

  double gain_{1.0};
};
}  // namespace planning
}  // namespace neodrive