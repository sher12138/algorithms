#pragma once

#include "motorway_speed_model_for_iter.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/math/hpipm_solver/motorway_speed_hpipm_solver_for_iter.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedMPCForIter {
 public:
  MotorwaySpeedMPCForIter() = delete;
  MotorwaySpeedMPCForIter(const double delta_t, const int n, const int nx,
                          const int nu, const int npc, const int ns,
                          const MotorwaySpeedForIter::State& init_state,
                          const MotorwaySpeedForIter::Control& init_control,
                          const double speed_limit,
                          const std::vector<double>& upper_s_bounds,
                          const std::vector<double>& lower_s_bounds,
                          const std::vector<double>& upper_v_bounds,
                          const std::vector<double>& lower_v_bounds,
                          const std::vector<double>& upper_a_bounds,
                          const std::vector<double>& lower_a_bounds,
                          const std::vector<double>& upper_jerk_bounds,
                          const std::vector<double>& lower_jerk_bounds,
                          const std::vector<STPoint>& goal_s,
                          const std::vector<STPoint>& goal_v,
                          const std::vector<STPoint>& goal_a);

  ~MotorwaySpeedMPCForIter();

  bool Process();

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(
      std::vector<MotorwaySpeedForIter::OptVariables>, optimal_solution)

 private:
  bool RunMPC();

  bool SetMPCProblem();

  bool SetStage(const MotorwaySpeedForIter::State& x_k,
                const MotorwaySpeedForIter::Control& u_k, const int step,
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
  MotorwaySpeedForIter::State init_state_{};
  MotorwaySpeedForIter::Control init_control_{};
  double speed_limit_{};
  std::vector<double> upper_s_bounds_{};
  std::vector<double> lower_s_bounds_{};
  std::vector<double> upper_v_bounds_{};
  std::vector<double> lower_v_bounds_{};
  std::vector<double> upper_a_bounds_{};
  std::vector<double> lower_a_bounds_{};
  std::vector<double> upper_jerk_bounds_{};
  std::vector<double> lower_jerk_bounds_{};
  std::vector<STPoint> goal_s_{};
  std::vector<STPoint> goal_v_{};
  std::vector<STPoint> goal_a_{};

  std::shared_ptr<MotorwaySpeedModelForIter> model_{};
  std::vector<MotorwaySpeedForIter::Stage> stages_{};
  std::vector<MotorwaySpeedForIter::OptVariables> optimal_solution_{};
  std::shared_ptr<MotorwaySpeedHpipmSolverForIter> mpc_solver_{};
};
}  // namespace planning
}  // namespace neodrive