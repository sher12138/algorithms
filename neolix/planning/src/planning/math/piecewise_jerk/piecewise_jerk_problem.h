#pragma once

#include <algorithm>
#include <tuple>
#include <utility>
#include <vector>

#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

/*
 * @brief:
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkProblem {
 public:
  PiecewiseJerkProblem(const std::size_t num_of_knots, const int max_iter,
                       const double delta_s, const double time_limit,
                       const std::array<double, 3> &x_init);

  // virtual ~PiecewiseJerkProblem() = default;
  virtual ~PiecewiseJerkProblem();

  void clean_all_data();

  void set_delta_s_vec(const std::vector<double> &delta_s_vec);

  void set_x_bounds(std::vector<std::pair<double, double>> x_bounds);
  void set_circles_x_bounds(std::vector<std::vector<std::pair<double, double>>>
                                circles_road_obs_boundary);

  void set_x_bounds(const double x_lower_bound, const double x_upper_bound);

  void set_dx_bounds(std::vector<std::pair<double, double>> dx_bounds);
  void set_dx_bounds(const double dx_lower_bound, const double dx_upper_bound);

  void set_ddx_bounds(std::vector<std::pair<double, double>> ddx_bounds);
  void set_ddx_bounds(const double ddx_lower_bound,
                      const double ddx_upper_bound);

  void set_dddx_bound(const double dddx_bound);
  void set_dddx_bound(const double dddx_lower_bound,
                      const double dddx_upper_bound);
  void set_dddx_bound(
      const std::vector<std::pair<double, double>> &dddx_bounds);

  void set_weight_x(const double weight_x);
  void set_weight_dx(const double weight_dx);
  void set_weight_ddx(const double weight_ddx);
  void set_weight_dddx(const double weight_dddx);

  void set_weight_soft_x(const double weight_soft_x);
  void set_value_soft_x(const double value_soft_x);

  void set_scale_factor(const std::array<double, 3> &scale_factor);

  void set_x_ref(const double weight_x_ref, std::vector<double> &x_ref);
  void set_x_ref(std::vector<double> &weight_x_ref_vec,
                 std::vector<double> &x_ref);

  void set_x_obs_dis(const double weight_x_obs_dis,
                     std::vector<double> &x_obs_dis_);

  void set_x_obs_dis(std::vector<double> &weight_x_obs_dis,
                     std::vector<double> &x_obs_dis);

  void set_end_state_ref(const std::array<double, 3> &weight_end_state,
                         const std::array<double, 3> &end_state_ref);

  void set_primal_warm_start(const std::vector<c_float> &primal_warm_start);

  void set_use_auto_warm_start(const bool use_auto_warm_start);

  std::vector<c_float> primal_warm_start() const;

  std::size_t num_of_knots() const;

  double DeltaS() const;

  double WeightX() const;
  double WeightDx() const;
  double WeightDdx() const;
  double WeightDddx() const;
  double WeightSoftX() const;
  double ValueSoftX() const;

  virtual bool Optimize(const bool use_inaccurate = false);

  const std::vector<double> &opt_x() const;
  const std::vector<double> &opt_dx() const;
  const std::vector<double> &opt_ddx() const;
  const std::vector<double> &opt_soft_x() const;

  void update_x_init(const std::array<double, 3> &x_init);

 protected:
  // naming convention follows osqp solver.
  virtual void CalculateKernel(std::vector<c_float> &P_data,
                               std::vector<c_int> &P_indices,
                               std::vector<c_int> &P_indptr) = 0;

  virtual void CalculateOffset(std::vector<c_float> &q) = 0;

  virtual void CalculateAffineConstraint(std::vector<c_float> &A_data,
                                         std::vector<c_int> &A_indices,
                                         std::vector<c_int> &A_indptr,
                                         std::vector<c_float> &lower_bounds,
                                         std::vector<c_float> &upper_bounds);
  virtual void CalculateAffineConstraintSoft(
      std::vector<c_float> &A_data, std::vector<c_int> &A_indices,
      std::vector<c_int> &A_indptr, std::vector<c_float> &lower_bounds,
      std::vector<c_float> &upper_bounds);

  virtual void update_bounds(std::vector<c_float> &lower_bounds,
                             std::vector<c_float> &upper_bounds);

  virtual OSQPSettings *SolverDefaultSettings();

  bool OptimizeWithOsqp(const bool use_inaccurate, const std::size_t kernel_dim,
                        const std::size_t num_affine_constraint,
                        std::vector<c_float> &P_data,
                        std::vector<c_int> &P_indices,  // NOLINT
                        std::vector<c_int> &P_indptr,
                        std::vector<c_float> &A_data,  // NOLINT
                        std::vector<c_int> &A_indices,
                        std::vector<c_int> &A_indptr,        // NOLINT
                        std::vector<c_float> &lower_bounds,  // NOLINT
                        std::vector<c_float> &upper_bounds,  // NOLINT
                        std::vector<c_float> &q);

  bool Optimize_with_auto_warm(const bool use_inaccurate);

  bool Optimize_without_auto_warm(const bool use_inaccurate);

 protected:
  std::vector<c_float> primal_warm_start_{};

  bool use_auto_warm_start_ = true;  // true means use

  std::size_t num_of_knots_ = 0;

  OSQPSettings *settings_ = nullptr;
  OSQPWorkspace *work_ = nullptr;  // Workspace
  OSQPData *data_ = nullptr;       // OSQPData

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;
  std::vector<double> soft_x_;

  std::array<double, 3> x_init_;

  std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::vector<std::pair<double, double>>> circles_x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;
  std::vector<std::pair<double, double>> dddx_bound_vec_;

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;

  double weight_soft_x_ = 0.0;

  double value_soft_x_ = 0.5;

  double delta_s_ = 1.0;
  std::vector<double> delta_s_vec_{};

  bool has_x_ref_ = false;
  double weight_x_ref_ = 0.0;
  std::vector<double> x_ref_;

  // un-uniformed weighting
  std::vector<double> weight_x_ref_vec_;

  // obs distance weight
  bool has_x_obs_dis_ = false;
  double weight_x_obs_dis_ = 0.0;
  std::vector<double> weight_x_obs_dis_vec_;
  std::vector<double> x_obs_dis_;

  bool has_end_state_ref_ = false;
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_state_ref_;

 private:
  const double kMaxVariableRange = 1.0e10;
  const std::size_t circles_num_ = 4;
};

}  // namespace planning
}  // namespace neodrive
