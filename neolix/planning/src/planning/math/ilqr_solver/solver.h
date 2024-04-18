#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cost_function.h"
#include "dynamic_model.h"
#include "gtest/gtest.h"
#include "problem.h"
#include "src/planning/common/optional.h"

namespace neodrive {
namespace planning {
namespace ilqr {

constexpr double kEpsilon = 1e-6;

// Solve the deterministic finite-horizon optimal control problem.
// minimize sum_over_i[cost(x_i,u_i)] + terminal_cost(x_n)
// s.t. x(i+1) = dynamics(x_i, u_i), min_u_i <= u_i <= max_u_i
// Template parameter X and U represent the state and action dimensions
// respectively.
template <int X, int U>
class Solver {
 public:
  using VecX = Eigen::Matrix<double, X, 1>;
  using VecU = Eigen::Matrix<double, U, 1>;
  using MatXX = Eigen::Matrix<double, X, X>;
  using MatUU = Eigen::Matrix<double, U, U>;
  using MatUX = Eigen::Matrix<double, U, X>;
  using MatXU = Eigen::Matrix<double, X, U>;
  using VectorOfVecU = std::vector<VecU, Eigen::aligned_allocator<VecU>>;
  using VectorOfVecX = std::vector<VecX, Eigen::aligned_allocator<VecX>>;
  using VectorOfMatUX = std::vector<MatUX, Eigen::aligned_allocator<MatUX>>;

  struct Config {
    double max_mu = 1e10;
    double min_mu = 1e-6;
    double delta_mu_factor = 1.6;
    double grad_exit_mu_thresh = 1e-5;
    int max_num_iter = 100;
    int max_num_qp_iter = 20;
    double min_grad_thresh = 1e-4;
    double line_search_min_cost_improvement_ratio = 0.0;
    double min_cost_improvement_threshold = 1e-3;
    int verbose_level = 0;
  };

  struct IterationSnapshot {
    int iter_num = 0;
    double total_cost = 0.0;
    std::unordered_map<std::string, double> cost_distribution;
    std::unordered_map<std::string, double> iter_meta;
    VectorOfVecX x_traj;
    VectorOfVecU u_traj;
  };

  struct Solution {
    bool is_solved = false;
    int final_iter_num = 0;
    double final_total_cost = 0.0;
    int selected_init_solution_index = 0;
    // TODO(xingxuetao): Change traj to trajectory.
    VectorOfVecX x_traj;
    VectorOfVecU u_traj;
    std::vector<IterationSnapshot> iter_snapshots;
  };

  explicit Solver(const Config& config);
  virtual ~Solver() = default;

  Solution Solve(const Problem<X, U>& problem) const;

 private:
  // Values computed by "np.power(10,np.linspace(0,-3,11))"
  std::vector<double> alpha_values_{
      1.,         0.50118723, 0.25118864, 0.12589254, 0.06309573, 0.03162278,
      0.01584893, 0.00794328, 0.00398107, 0.00199526, 0.001};
  struct IterationMeta {
    int iter_num = 0;
    double mu = 1.0;
    double delta_mu = 1.0;
    double alpha = 1.0;
    double cost_improvement = 0.0;
    double cost_improvement_ratio = 0.0;
    double expected_cost_improvement = 0.0;
    double normalized_grad = 0.0;
    bool is_backward_succeeded = false;
    bool is_forward_succeeded = false;
    std::vector<int> clamped_action_indexes;
    // H_uu_ff_inv_map is not declared locally but here in meta to prevent
    // frequent allocation of dynamic memory.
    std::unordered_map<unsigned int, MatUU> H_uu_ff_inv_map;
    int final_quadratic_iter_num = 0;
    VectorOfVecU k_traj;
    VectorOfMatUX K_traj;
    std::array<double, 2> dV = {0.0, 0.0};
    std::unordered_map<std::string, double> cost_map;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct ElementaryActionLimit {
    int index_in_action = -1;
    double limit_value = 0.0;
  };

  struct ActionLimit {
    std::vector<ElementaryActionLimit> lower_limits;
    std::vector<ElementaryActionLimit> upper_limits;
  };

  double max_mu_ = 1e10;
  double min_mu_ = 1e-6;
  double delta_mu_factor_ = 1.6;
  double grad_exit_mu_thresh_ = 1e-5;
  int max_num_iter_ = 100;
  int max_num_qp_iter_ = 20;
  double min_grad_thresh_ = 1e-4;
  double line_search_min_cost_improvement_ratio_ = 0.0;
  double min_cost_improvement_threshold_ = 1e-3;
  double min_expect_cost_improvement_threshold_ = 1e-5;
  double min_cost_decreasing_threshold_ = -1e-7;
  // 0=>low, 1=>medium, 2=>high
  int verbose_level_ = 0;

  void ForwardIteration(const Problem<X, U>& problem,
                        const IterationMeta& context,
                        const VectorOfVecX& x_traj, const VectorOfVecU& u_traj,
                        const base::Optional<ActionLimit>& action_limit,
                        VectorOfVecX* x_traj_updated,
                        VectorOfVecU* u_traj_updated) const;

  bool BackwardIteration(const Problem<X, U>& problem,
                         const VectorOfVecX& x_traj, const VectorOfVecU& u_traj,
                         const base::Optional<ActionLimit>& action_limit,
                         IterationMeta* meta) const;

  double FindBestInitSolution(
      const Problem<X, U>& problem, int* index, VectorOfVecX* x_traj,
      VectorOfVecU* u_traj,
      std::vector<IterationSnapshot>* nullable_snapshots) const;

  VectorOfVecX RollOutTraj(const Problem<X, U>& problem,
                           const VectorOfVecU& u_traj) const;

  double ComputeCost(
      const Problem<X, U>& problem, const VectorOfVecX& x_traj,
      const VectorOfVecU& u_traj,
      std::unordered_map<std::string, double>* nullable_cost_map) const;

  void IncreaseRegularization(IterationMeta* meta) const;

  void ReduceRegularization(IterationMeta* meta) const;

  double ComputeNormalizedGrad(const VectorOfVecU& k_traj,
                               const VectorOfVecU& u_traj) const;

  void UpdateSnapshot(const IterationMeta& meta, const VectorOfVecX& x_traj,
                      const VectorOfVecU& u_traj,
                      IterationSnapshot* snapshot) const;

  base::Optional<ActionLimit> PrepareActionLimit(const Problem<X, U>& problem,
                                                 IterationMeta* meta) const;

  // Find the optimal du minimizing 0.5 * du.transpose() * H_uu * du + q_u * du,
  // s.t. min_u <= u + du <= max_u.
  base::Optional<VecU> OptimizeActionModification(
      const MatUU& H_uu, const VecU& q_u, const VecU& u,
      const ActionLimit& action_limit, MatUU* H_uu_ff_inv,
      IterationMeta* meta) const;

  void ClampActionModification(const VecU& u, const ActionLimit& action_limit,
                               const VecU& min_du, const VecU& max_du,
                               VecU* du) const;

  void ClampAction(const ActionLimit& action_limit, VecU* u) const;

  FRIEND_TEST(SolverTest, ProjectedNewton);
};

// Below are implementations.
template <int X, int U>
Solver<X, U>::Solver(const Config& config)
    : max_mu_(config.max_mu),
      min_mu_(config.min_mu),
      delta_mu_factor_(config.delta_mu_factor),
      grad_exit_mu_thresh_(config.grad_exit_mu_thresh),
      max_num_iter_(config.max_num_iter),
      max_num_qp_iter_(config.max_num_qp_iter),
      min_grad_thresh_(config.min_grad_thresh),
      line_search_min_cost_improvement_ratio_(
          config.line_search_min_cost_improvement_ratio),
      min_cost_improvement_threshold_(config.min_cost_improvement_threshold),
      verbose_level_(config.verbose_level) {}

template <int X, int U>
void Solver<X, U>::ForwardIteration(
    const Problem<X, U>& problem, const IterationMeta& context,
    const VectorOfVecX& x_traj, const VectorOfVecU& u_traj,
    const base::Optional<ActionLimit>& action_limit,
    VectorOfVecX* x_traj_updated, VectorOfVecU* u_traj_updated) const {
  CHECK(x_traj_updated != nullptr);
  CHECK(u_traj_updated != nullptr);
  x_traj_updated->resize(x_traj.size());
  u_traj_updated->resize(u_traj.size());
  (*x_traj_updated)[0] = x_traj[0];
  const int num_steps = problem.num_steps();
  for (int i = 0; i < num_steps; ++i) {
    (*u_traj_updated)[i] =
        u_traj[i] + context.alpha * context.k_traj[i] +
        context.K_traj[i] * ((*x_traj_updated)[i] - x_traj[i]);
    if (action_limit) {
      ClampAction(*action_limit, &((*u_traj_updated)[i]));
    }
    (*x_traj_updated)[i + 1] = problem.dynamic_model(i).Evaluate(
        (*x_traj_updated)[i], (*u_traj_updated)[i]);
  }
}

template <int X, int U>
bool Solver<X, U>::BackwardIteration(
    const Problem<X, U>& problem, const VectorOfVecX& x_traj,
    const VectorOfVecU& u_traj, const base::Optional<ActionLimit>& action_limit,
    IterationMeta* meta) const {
  const int num_steps = problem.num_steps();
  // We follow the variable namings in the paper "control-limited DDP", so some
  // variable namings violates our code style. V is the optimal cost-to-go at a
  // specific step l is the cost term associated with a specific step Q is the
  // pseudo-Hamiltonian at a specific step EvaluateCost is the dynamic function
  // at a specific step The subscripts x and u represent the derivative w.r.t x
  // and u
  typename CostFunction<X, U>::Derivatives terminal_cost_derivatives;
  problem.terminal_cost_function().EvaluateDerivatives(
      x_traj[num_steps], VecU::Zero(), &terminal_cost_derivatives);
  MatXX V_xx = terminal_cost_derivatives.d2fdxdx;
  VecX V_x = terminal_cost_derivatives.dfdx;
  meta->k_traj.resize(num_steps);
  meta->K_traj.resize(num_steps);
  meta->dV = {0.0, 0.0};
  for (int i = num_steps - 1; i >= 0; --i) {
    typename CostFunction<X, U>::Derivatives cost_derivatives;
    problem.cost_function(i).EvaluateDerivatives(x_traj[i], u_traj[i],
                                                 &cost_derivatives);
    const VecX& l_x = cost_derivatives.dfdx;
    const MatXX& l_xx = cost_derivatives.d2fdxdx;
    const VecU& l_u = cost_derivatives.dfdu;
    const MatUU& l_uu = cost_derivatives.d2fdudu;
    const MatUX& l_ux = cost_derivatives.d2fdudx;
    typename DynamicModel<X, U>::Jacobians jacobians =
        problem.dynamic_model(i).EvaluateJacobians(x_traj[i], u_traj[i]);
    const MatXX& f_x = jacobians.dfdx;
    const MatXU& f_u = jacobians.dfdu;
    VecX Q_x = l_x + f_x.transpose() * V_x;
    VecU Q_u = l_u + f_u.transpose() * V_x;
    MatXX Q_xx = l_xx + f_x.transpose() * V_xx * f_x;
    MatUU Q_uu = l_uu + f_u.transpose() * V_xx * f_u;
    MatUX Q_ux = l_ux + f_u.transpose() * V_xx * f_x;
    MatUU Q_uu_reg =
        l_uu + f_u.transpose() * V_xx * f_u + meta->mu * MatUU::Identity();

    if (action_limit) {
      MatUU Q_uu_reg_ff_inv;
      base::Optional<VecU> optimal_du = OptimizeActionModification(
          Q_uu_reg, Q_u, u_traj[i], *action_limit, &Q_uu_reg_ff_inv, meta);
      if (!optimal_du) {
        return false;
      }
      meta->k_traj[i] = *optimal_du;
      MatUX Q_ux_f = Q_ux;
      for (const int index : meta->clamped_action_indexes) {
        Q_ux_f.row(index) = VecX::Zero().transpose();
      }
      meta->K_traj[i] = -Q_uu_reg_ff_inv * Q_ux_f;
    } else {
      Eigen::LLT<MatUU> llt_of_Q_uu_reg(Q_uu_reg);
      if (llt_of_Q_uu_reg.info() == Eigen::NumericalIssue) {
        return false;
      }
      MatUU Q_uu_reg_chol = llt_of_Q_uu_reg.matrixL().transpose();
      MatUU Q_uu_reg_inv =
          Q_uu_reg_chol.inverse() * Q_uu_reg_chol.transpose().inverse();
      meta->k_traj[i] = -Q_uu_reg_inv * Q_u;
      meta->K_traj[i] = -Q_uu_reg_inv * Q_ux;
    }
    meta->dV[0] += meta->k_traj[i].transpose() * Q_u;
    meta->dV[1] +=
        (0.5 * meta->k_traj[i].transpose() * Q_uu * meta->k_traj[i]).value();
    V_x = Q_x + meta->K_traj[i].transpose() * Q_uu * meta->k_traj[i] +
          meta->K_traj[i].transpose() * Q_u +
          Q_ux.transpose() * meta->k_traj[i];
    V_xx = Q_xx + meta->K_traj[i].transpose() * Q_uu * meta->K_traj[i] +
           meta->K_traj[i].transpose() * Q_ux +
           Q_ux.transpose() * meta->K_traj[i];
    V_xx = 0.5 * (V_xx + V_xx.transpose());
  }

  return true;
}

template <int X, int U>
void Solver<X, U>::IncreaseRegularization(IterationMeta* meta) const {
  meta->delta_mu =
      std::max(delta_mu_factor_, meta->delta_mu * delta_mu_factor_);
  meta->mu = std::max(min_mu_, meta->mu * meta->delta_mu);
}

template <int X, int U>
void Solver<X, U>::ReduceRegularization(IterationMeta* meta) const {
  meta->delta_mu =
      std::min(1.0 / delta_mu_factor_, meta->delta_mu / delta_mu_factor_);
  if (meta->mu * meta->delta_mu > min_mu_) {
    meta->mu *= meta->delta_mu;
  } else {
    meta->mu = 0.0;
  }
}

template <int X, int U>
double Solver<X, U>::ComputeNormalizedGrad(const VectorOfVecU& k_traj,
                                           const VectorOfVecU& u_traj) const {
  CHECK_EQ(k_traj.size(), u_traj.size());
  double accumulated_grad = 0.0;
  for (int i = 0; i < static_cast<int>(k_traj.size()); ++i) {
    accumulated_grad +=
        (k_traj[i].cwiseAbs().array() / (u_traj[i].cwiseAbs().array() + 1.0))
            .maxCoeff();
  }
  return accumulated_grad / k_traj.size();
}

template <int X, int U>
typename Solver<X, U>::VectorOfVecX Solver<X, U>::RollOutTraj(
    const Problem<X, U>& problem, const VectorOfVecU& u_traj) const {
  const int num_steps = problem.num_steps();
  CHECK_EQ(num_steps, u_traj.size());
  VectorOfVecX x_traj;
  x_traj.reserve(num_steps + 1);
  x_traj.push_back(problem.x0());
  for (int i = 0; i < num_steps; ++i) {
    x_traj.push_back(problem.dynamic_model(i).Evaluate(x_traj[i], u_traj[i]));
  }
  return x_traj;
}

template <int X, int U>
double Solver<X, U>::ComputeCost(
    const Problem<X, U>& problem, const VectorOfVecX& x_traj,
    const VectorOfVecU& u_traj,
    std::unordered_map<std::string, double>* nullable_cost_map) const {
  const int num_steps = problem.num_steps();
  CHECK_EQ(x_traj.size(), num_steps + 1);
  CHECK_EQ(u_traj.size(), num_steps);
  double cost = 0.0;
  for (int i = 0; i < num_steps; ++i) {
    cost += problem.cost_function(i).EvaluateCost(x_traj[i], u_traj[i],
                                                  nullable_cost_map);
  }
  // Terminal cost depends on only state value.
  cost += problem.terminal_cost_function().EvaluateCost(
      x_traj[num_steps], VecU::Zero(), nullable_cost_map);
  CHECK(!std::isnan(cost));
  CHECK(std::isfinite(cost));
  return cost;
}

template <int X, int U>
double Solver<X, U>::FindBestInitSolution(
    const Problem<X, U>& problem, int* index, VectorOfVecX* x_traj,
    VectorOfVecU* u_traj,
    std::vector<IterationSnapshot>* nullable_snapshots) const {
  CHECK(index != nullptr);
  CHECK(x_traj != nullptr);
  CHECK(u_traj != nullptr);
  const std::vector<VectorOfVecU>& init_solutions =
      problem.candidate_init_solutions();
  double min_cost = std::numeric_limits<double>::infinity();
  VectorOfVecU u_traj_candidate;
  VectorOfVecX x_traj_candidate;
  for (int i = 0; i < static_cast<int>(init_solutions.size()); ++i) {
    const VectorOfVecU& u_traj_i = init_solutions[i];
    VectorOfVecX x_traj_i = RollOutTraj(problem, u_traj_i);
    std::unordered_map<std::string, double> cost_map;
    double cost = std::numeric_limits<double>::max();
    if (verbose_level_ > 1) {
      cost = ComputeCost(problem, x_traj_i, u_traj_i, &cost_map);
    } else {
      cost = ComputeCost(problem, x_traj_i, u_traj_i, nullptr);
    }
    if (verbose_level_ > 1 && nullable_snapshots != nullptr) {
      IterationSnapshot init_solution_snapshot;
      init_solution_snapshot.iter_num = -1;
      init_solution_snapshot.total_cost = cost;
      init_solution_snapshot.cost_distribution = std::move(cost_map);
      init_solution_snapshot.x_traj = x_traj_i;
      init_solution_snapshot.u_traj = u_traj_i;
      init_solution_snapshot.total_cost = cost;
      nullable_snapshots->push_back(std::move(init_solution_snapshot));
    }
    // save the first solution to make sure initialization of x_traj and u_traj
    if (cost < min_cost) {
      min_cost = cost;
      *index = i;
      *x_traj = std::move(x_traj_i);
      *u_traj = u_traj_i;
    }
  }
  return min_cost;
}

template <int X, int U>
void Solver<X, U>::UpdateSnapshot(const IterationMeta& meta,
                                  const VectorOfVecX& x_traj,
                                  const VectorOfVecU& u_traj,
                                  IterationSnapshot* snapshot) const {
  snapshot->iter_meta["alpha"] = meta.alpha;
  snapshot->iter_meta["cost_improvement"] = meta.cost_improvement;
  snapshot->iter_meta["expected_cost_improvement"] =
      meta.expected_cost_improvement;
  snapshot->iter_meta["cost_improvement_ratio"] = meta.cost_improvement_ratio;
  snapshot->iter_meta["normalized_grad"] = meta.normalized_grad;
  // meta.mu is the mu value for next iteration. We divide by delta_mu to get
  // the mu for current iteration.
  snapshot->iter_meta["mu"] = meta.mu / meta.delta_mu;
  snapshot->iter_meta["delta_mu"] = meta.delta_mu;
  snapshot->iter_meta["is_backward_succeeded"] = meta.is_backward_succeeded;
  snapshot->iter_meta["is_forward_succeeded"] = meta.is_forward_succeeded;
  // save results of iteration
  snapshot->iter_num = meta.iter_num;
  snapshot->cost_distribution = meta.cost_map;
  snapshot->x_traj = x_traj;
  snapshot->u_traj = u_traj;
}

template <int X, int U>
base::Optional<typename Solver<X, U>::ActionLimit>
Solver<X, U>::PrepareActionLimit(const Problem<X, U>& problem,
                                 IterationMeta* meta) const {
  ActionLimit action_limit;
  action_limit.lower_limits.reserve(U);
  action_limit.upper_limits.reserve(U);
  for (int i = 0; i < U; ++i) {
    CHECK_LE(problem.min_u()(i), problem.max_u()(i));
    if (std::isfinite(problem.min_u()(i))) {
      action_limit.lower_limits.push_back(
          {.index_in_action = i, .limit_value = problem.min_u()(i)});
    }
    if (std::isfinite(problem.max_u()(i))) {
      action_limit.upper_limits.push_back(
          {.index_in_action = i, .limit_value = problem.max_u()(i)});
    }
  }
  if (action_limit.lower_limits.empty() && action_limit.upper_limits.empty()) {
    return base::none;
  }
  meta->clamped_action_indexes.reserve(U);
  static_assert(U <= 31,
                "A control-limited problem with more than 31 control "
                "dimentions is not supported.");
  unsigned int num_possible_clamps = 1 << U;
  CHECK_LE(num_possible_clamps, meta->H_uu_ff_inv_map.max_size());
  meta->H_uu_ff_inv_map.reserve(num_possible_clamps);
  return action_limit;
}

template <int X, int U>
base::Optional<typename Solver<X, U>::VecU>
Solver<X, U>::OptimizeActionModification(const MatUU& H_uu, const VecU& q_u,
                                         const VecU& u,
                                         const ActionLimit& action_limit,
                                         MatUU* H_uu_ff_inv,
                                         IterationMeta* meta) const {
  VecU min_du = VecU::Zero();
  VecU max_du = VecU::Zero();
  for (const ElementaryActionLimit& elementary_lower_limit :
       action_limit.lower_limits) {
    const int index = elementary_lower_limit.index_in_action;
    min_du(index) = elementary_lower_limit.limit_value - u(index);
  }
  for (const ElementaryActionLimit& elementary_upper_limit :
       action_limit.upper_limits) {
    const int index = elementary_upper_limit.index_in_action;
    max_du(index) = elementary_upper_limit.limit_value - u(index);
  }
  meta->H_uu_ff_inv_map.clear();
  unsigned int binary_key =
      0;  // with clamped-index bits being 1, free-index bits being 0

  VecU du = VecU::Zero();
  ClampActionModification(u, action_limit, min_du, max_du, &du);
  double quadratic_cost =
      (0.5 * du.transpose() * H_uu * du + q_u.transpose() * du).value();
  for (int iter = 0; iter < max_num_qp_iter_; ++iter) {
    const VecU gradient = q_u + H_uu * du;
    meta->clamped_action_indexes.clear();
    binary_key = 0;
    for (const ElementaryActionLimit& elementary_lower_limit :
         action_limit.lower_limits) {
      const int index = elementary_lower_limit.index_in_action;
      if (gradient(index) > 0 && du(index) < min_du(index) + kEpsilon) {
        meta->clamped_action_indexes.emplace_back(index);
        binary_key |= 1 << index;
      }
    }
    for (const ElementaryActionLimit& elementary_upper_limit :
         action_limit.upper_limits) {
      const int index = elementary_upper_limit.index_in_action;
      if (gradient(index) < 0 && du(index) > max_du(index) - kEpsilon) {
        meta->clamped_action_indexes.emplace_back(index);
        binary_key |= 1 << index;
      }
    }
    // Matrix subscripts: a for all, c for clamped, f for free.
    // For example, H_uu_ac is a matrix with all rows and clamped columns from
    // H_uu (other entries being zero).
    VecU du_c = VecU::Zero();
    VecU q_u_c = VecU::Zero();
    MatUU H_uu_ac = MatUU::Zero();
    MatUU H_uu_cc = MatUU::Zero();
    MatUU H_uu_ff = H_uu;
    VecU gradient_f = gradient;
    for (const int index : meta->clamped_action_indexes) {
      du_c(index) = du(index);
      q_u_c(index) = q_u(index);
      H_uu_ac.col(index) = H_uu.col(index);
      H_uu_ff.row(index) = VecU::Zero().transpose();
      H_uu_ff.col(index) = VecU::Zero();
      H_uu_ff(index, index) = 1.0;
      gradient_f(index) = 0.0;
      for (const int internal_index : meta->clamped_action_indexes) {
        H_uu_cc(internal_index, index) = H_uu(internal_index, index);
      }
    }
    const auto iterator = meta->H_uu_ff_inv_map.find(binary_key);
    if (iterator == meta->H_uu_ff_inv_map.end()) {
      Eigen::LLT<MatUU> llt_of_H_uu_ff(H_uu_ff);
      if (llt_of_H_uu_ff.info() == Eigen::NumericalIssue) {
        return base::none;
      }
      MatUU H_uu_ff_chol = llt_of_H_uu_ff.matrixL().transpose();
      *H_uu_ff_inv =
          H_uu_ff_chol.inverse() * H_uu_ff_chol.transpose().inverse();
      meta->H_uu_ff_inv_map[binary_key] = *H_uu_ff_inv;
    } else {
      *H_uu_ff_inv = iterator->second;
    }
    if (gradient_f.norm() < min_grad_thresh_) {
      meta->final_quadratic_iter_num = iter;
      return du;
    }
    VecU du_newton_step =
        -(*H_uu_ff_inv) * (q_u - q_u_c + (H_uu_ac - H_uu_cc) * du_c) -
        (du - du_c);
    VecU next_du = VecU::Zero();
    double next_quadratic_cost = 0.0;
    double expected_quadratic_cost_improvement = 0.0;
    double cost_quadratic_improvement_ratio = 0.0;
    for (const double alpha : alpha_values_) {
      next_du = du + alpha * du_newton_step;
      ClampActionModification(u, action_limit, min_du, max_du, &next_du);
      next_quadratic_cost = (0.5 * next_du.transpose() * H_uu * next_du +
                             q_u.transpose() * next_du)
                                .value();
      expected_quadratic_cost_improvement =
          gradient.transpose() * (du - next_du);
      if (expected_quadratic_cost_improvement > 0.0) {
        cost_quadratic_improvement_ratio =
            (quadratic_cost - next_quadratic_cost) /
            expected_quadratic_cost_improvement;
      } else {
        cost_quadratic_improvement_ratio =
            std::copysign(1.0, quadratic_cost - next_quadratic_cost);
      }
      if (cost_quadratic_improvement_ratio >
          line_search_min_cost_improvement_ratio_) {
        break;
      }
    }
    du = next_du;
    quadratic_cost = next_quadratic_cost;
  }
  return base::none;
}

template <int X, int U>
void Solver<X, U>::ClampActionModification(const VecU& u,
                                           const ActionLimit& action_limit,
                                           const VecU& min_du,
                                           const VecU& max_du, VecU* du) const {
  for (const ElementaryActionLimit& elementary_lower_limit :
       action_limit.lower_limits) {
    const int index = elementary_lower_limit.index_in_action;
    (*du)(index) = std::max((*du)(index), min_du(index));
  }
  for (const ElementaryActionLimit& elementary_upper_limit :
       action_limit.upper_limits) {
    const int index = elementary_upper_limit.index_in_action;
    (*du)(index) = std::min((*du)(index), max_du(index));
  }
}

template <int X, int U>
void Solver<X, U>::ClampAction(const ActionLimit& action_limit, VecU* u) const {
  for (const ElementaryActionLimit& elementary_lower_limit :
       action_limit.lower_limits) {
    const int index = elementary_lower_limit.index_in_action;
    (*u)(index) = std::max((*u)(index), elementary_lower_limit.limit_value);
  }
  for (const ElementaryActionLimit& elementary_upper_limit :
       action_limit.upper_limits) {
    const int index = elementary_upper_limit.index_in_action;
    (*u)(index) = std::min((*u)(index), elementary_upper_limit.limit_value);
  }
}

template <int X, int U>
typename Solver<X, U>::Solution Solver<X, U>::Solve(
    const Problem<X, U>& problem) const {
  VectorOfVecX x_traj;
  VectorOfVecU u_traj;
  Solution solution;
  double init_cost;
  if (verbose_level_ > 1) {
    std::vector<IterationSnapshot> nullable_snapshots;
    init_cost =
        FindBestInitSolution(problem, &(solution.selected_init_solution_index),
                             &x_traj, &u_traj, &nullable_snapshots);
    solution.iter_snapshots.insert(solution.iter_snapshots.end(),
                                   nullable_snapshots.begin(),
                                   nullable_snapshots.end());
  } else {
    init_cost =
        FindBestInitSolution(problem, &(solution.selected_init_solution_index),
                             &x_traj, &u_traj, nullptr);
  }
  CHECK_EQ(x_traj.size(), problem.num_steps() + 1);
  CHECK_EQ(u_traj.size(), problem.num_steps());
  std::vector<double> cost_history;
  cost_history.reserve(max_num_iter_);
  cost_history.push_back(init_cost);

  IterationMeta meta;
  base::Optional<ActionLimit> action_limit = PrepareActionLimit(problem, &meta);
  int iter = 0;
  for (; iter < max_num_iter_; ++iter) {
    meta.is_backward_succeeded = false;
    meta.is_forward_succeeded = false;
    base::Optional<IterationSnapshot> snapshot;
    if (verbose_level_ > 0) {
      snapshot.emplace();
      snapshot->total_cost = cost_history.back();
      meta.iter_num = iter;
    }
    bool is_backward_succeeded = false;
    while (meta.mu < max_mu_) {
      is_backward_succeeded =
          BackwardIteration(problem, x_traj, u_traj, action_limit, &meta);
      if (is_backward_succeeded) {
        meta.is_backward_succeeded = is_backward_succeeded;
        break;
      }
      IncreaseRegularization(&meta);
    }

    meta.normalized_grad = ComputeNormalizedGrad(meta.k_traj, u_traj);
    if (meta.normalized_grad < min_grad_thresh_ &&
        meta.mu < grad_exit_mu_thresh_) {
      // Stop iteration due to small gradient.
      solution.is_solved = true;
      break;
    }

    bool is_forward_succeeded = false;
    VectorOfVecX x_traj_updated;
    VectorOfVecU u_traj_updated;
    double cost_updated = 0.0;
    if (is_backward_succeeded) {
      // Backtracking line search
      std::unordered_map<std::string, double> cost_map;
      for (double alpha : alpha_values_) {
        cost_map.clear();
        meta.alpha = alpha;
        ForwardIteration(problem, meta, x_traj, u_traj, action_limit,
                         &x_traj_updated, &u_traj_updated);
        if (verbose_level_ > 1) {
          cost_updated =
              ComputeCost(problem, x_traj_updated, u_traj_updated, &cost_map);
        } else {
          cost_updated =
              ComputeCost(problem, x_traj_updated, u_traj_updated, nullptr);
        }
        meta.cost_improvement = cost_history.back() - cost_updated;
        meta.expected_cost_improvement =
            -meta.alpha * (meta.dV[0] + meta.alpha * meta.dV[1]);
        if (meta.expected_cost_improvement >
            min_expect_cost_improvement_threshold_) {
          meta.cost_improvement_ratio =
              meta.cost_improvement / meta.expected_cost_improvement;
        } else if (meta.expected_cost_improvement > 0.0) {
          meta.cost_improvement_ratio =
              meta.cost_improvement > min_cost_decreasing_threshold_
                  ? 1.0
                  : meta.cost_improvement;
        } else {
          meta.cost_improvement_ratio =
              std::copysign(1.0, meta.cost_improvement);
        }
        if (meta.cost_improvement_ratio >
            line_search_min_cost_improvement_ratio_) {
          is_forward_succeeded = true;
          meta.is_forward_succeeded = is_forward_succeeded;
          break;
        }
      }
      if (verbose_level_ > 1) {
        // Save cost distribution to meta
        meta.cost_map = std::move(cost_map);
      }
    }

    if (is_forward_succeeded) {
      x_traj = std::move(x_traj_updated);
      u_traj = std::move(u_traj_updated);
      cost_history.push_back(cost_updated);
      if (meta.cost_improvement < min_cost_improvement_threshold_) {
        // Converged, exit
        solution.is_solved = true;
        break;
      }
      ReduceRegularization(&meta);
    } else {
      IncreaseRegularization(&meta);
      if (meta.mu > max_mu_) {
        // Regularization exceeds maximum, exit.
        solution.is_solved = false;
        break;
      }
    }

    // Update snapshot
    if (verbose_level_ > 1) {
      snapshot->total_cost = cost_history.back();
      UpdateSnapshot(meta, x_traj, u_traj, &snapshot.value());
    }
    if (snapshot.is_initialized()) {
      solution.iter_snapshots.push_back(std::move(snapshot.value()));
    }
  }
  solution.final_iter_num = std::min(iter, max_num_iter_ - 1);
  solution.final_total_cost = cost_history.back();
  if (verbose_level_ > 0) {
    IterationSnapshot snapshot;
    snapshot.total_cost = cost_history.back();
    if (verbose_level_ > 1) {
      UpdateSnapshot(meta, x_traj, u_traj, &snapshot);
    }
    solution.iter_snapshots.push_back(std::move(snapshot));
  }

  bool log_output = false;
  if (verbose_level_ > 1 && log_output) {
    for (const auto& snapshot : solution.iter_snapshots) {
      // Choose to use cout because we want to keep the printing compact and
      // pretty.
      std::cout << std::setprecision(5) << "iter=" << snapshot.iter_num
                << ", cost=" << snapshot.total_cost << ", ";
      for (const auto& name_and_val : snapshot.iter_meta) {
        std::cout << name_and_val.first << "=" << name_and_val.second << ", ";
      }
      for (const auto& cost : snapshot.cost_distribution) {
        std::cout << "cost_name=" << cost.first << ", val=" << cost.second
                  << ", ";
      }
      std::cout << "\n";
    }
  }
  solution.x_traj = std::move(x_traj);
  solution.u_traj = std::move(u_traj);
  return solution;
}

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
