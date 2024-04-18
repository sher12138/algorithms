#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class FemPosOsqpSolver {
 public:
  FemPosOsqpSolver() = default;

  virtual ~FemPosOsqpSolver() = default;

  bool Solve();

  void set_ref_points(const std::vector<std::pair<double, double>>& ref_points);

  void set_bounds_around_refs(const std::vector<double>& bounds_around_refs);

  void set_weight_fem_pos_deviation(const double weight_fem_pos_deviation);

  void set_weight_path_length(const double weight_path_length);

  void set_weight_ref_deviation(const double weight_ref_deviation);

  void set_max_iter(const int max_iter);

  void set_time_limit(const double time_limit);

  void set_verbose(const bool verbose);

  void set_scaled_termination(const bool scaled_termination);

  void set_warm_start(const bool warm_start);

  const std::vector<double>& opt_x() const;

  const std::vector<double>& opt_y() const;

 private:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);

  void CalculateOffset(std::vector<c_float>* q);

  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);

  void SetPrimalWarmStart(std::vector<c_float>* primal_warm_start);

  bool OptimizeWithOsqp(
      const std::size_t kernel_dim, const std::size_t num_affine_constraint,
      std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
      std::vector<c_int>* P_indptr, std::vector<c_float>* A_data,
      std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr,
      std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds,
      std::vector<c_float>* q, std::vector<c_float>* primal_warm_start,
      OSQPData* data, OSQPWorkspace** work, OSQPSettings* settings);

 private:
  // Reference points and deviation bounds
  std::vector<std::pair<double, double>> ref_points_;
  std::vector<double> bounds_around_refs_;

  // Weights in optimization cost function
  double weight_fem_pos_deviation_ = 1.0e5;
  double weight_ref_deviation_ = 1.0;
  double weight_path_length_ = 1.0;

  // Settings of osqp
  int max_iter_ = 4000;
  double time_limit_ = 0.0;
  bool verbose_ = false;
  bool scaled_termination_ = true;
  bool warm_start_ = true;

  // Optimization problem definitions
  std::size_t num_of_points_ = 0;
  int num_of_variables_ = 0;
  int num_of_constraints_ = 0;

  // Optimized_result
  std::vector<double> x_;
  std::vector<double> y_;
};
}  // namespace planning
}  // namespace neodrive
