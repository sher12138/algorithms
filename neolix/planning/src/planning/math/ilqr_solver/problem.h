#pragma once

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "cost_function.h"
#include "dynamic_model.h"
#include "src/planning/common/planning_logger.h"

namespace neodrive {
namespace planning {
namespace ilqr {

// Defines an iLQR problem
// Example code:
//
// Problem problem(x0);
// for (int i=0; i<n; ++i) {
//   problem.AddStep(std::move(cost_function_i), std::move(dynamic_model_i));
// }
// problem.AddTerminalCost(cost_function_n);
//
// In the above code, x0 represents the initial state.
// cost_function_i represents the cost definition at step i, and its inputs are
// (xi,ui) which is the state-action pair at step i. dynamic_model_i represents
// the transfer function at step i. cost_function_n is the cost function for the
// terminal state, and it defines how we want to position the end point of the
// trajectory.
template <int X, int U>
class Problem final {
 public:
  using VecX = Eigen::Matrix<double, X, 1>;
  using VecU = Eigen::Matrix<double, U, 1>;
  using VectorOfVecU = std::vector<VecU, Eigen::aligned_allocator<VecU>>;

  explicit Problem(const VecX& x0) : x0_(x0) {}

  void AddStep(std::unique_ptr<CostFunction<X, U>> cost,
               std::unique_ptr<DynamicModel<X, U>> model) {
    cost_functions_.push_back(std::move(cost));
    dynamic_models_.push_back(std::move(model));
    ++num_steps_;
  }

  void AddTerminalCost(std::unique_ptr<CostFunction<X, U>> cost) {
    terminal_cost_function_ = std::move(cost);
  }

  void AddCandidateInitSolution(VectorOfVecU init_solution) {
    CHECK_EQ(init_solution.size(), num_steps_);
    candidate_init_solutions_.push_back(std::move(init_solution));
  }

  const CostFunction<X, U>& cost_function(int i) const {
    CHECK_GE(i, 0);
    CHECK_LT(i, num_steps_);
    return *cost_functions_[i];
  }

  CostFunction<X, U>* mutable_cost_function(int i) {
    CHECK_GE(i, 0);
    CHECK_LT(i, num_steps_);
    return cost_functions_[i].get();
  }

  const DynamicModel<X, U>& dynamic_model(int i) const {
    CHECK_GE(i, 0);
    CHECK_LT(i, num_steps_);
    return *dynamic_models_[i];
  }

  DynamicModel<X, U>* mutable_dynamic_model(int i) const {
    CHECK_GE(i, 0);
    CHECK_LT(i, num_steps_);
    return dynamic_models_[i].get();
  }

  const CostFunction<X, U>& terminal_cost_function() const {
    CHECK(terminal_cost_function_ != nullptr);
    return *terminal_cost_function_;
  }

  CostFunction<X, U>* mutable_terminal_cost_function() const {
    CHECK(terminal_cost_function_ != nullptr);
    return terminal_cost_function_.get();
  }

  const VecX& x0() const { return x0_; }

  VecX* mutable_x0() { return &x0_; }

  int num_steps() const { return num_steps_; }

  const VecU& min_u() const { return min_u_; }

  VecU* mutable_min_u() { return &min_u_; }

  const VecU& max_u() const { return max_u_; }

  VecU* mutable_max_u() { return &max_u_; }

  const std::vector<VectorOfVecU>& candidate_init_solutions() const {
    CHECK(candidate_init_solutions_.empty() != true);
    return candidate_init_solutions_;
  }

  std::vector<VectorOfVecU>* mutable_candidate_init_solutions() {
    return &candidate_init_solutions_;
  }

 private:
  int num_steps_ = 0;
  VecX x0_ = VecX::Zero();
  VecU min_u_ = VecU::Constant(-std::numeric_limits<double>::infinity());
  VecU max_u_ = VecU::Constant(std::numeric_limits<double>::infinity());
  std::vector<std::vector<VecU, Eigen::aligned_allocator<VecU>>>
      candidate_init_solutions_;
  std::vector<std::unique_ptr<CostFunction<X, U>>> cost_functions_;
  std::vector<std::unique_ptr<DynamicModel<X, U>>> dynamic_models_;
  std::unique_ptr<CostFunction<X, U>> terminal_cost_function_ = nullptr;
};

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
