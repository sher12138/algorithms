#pragma once

namespace neodrive {
namespace planning {

class FemPosConfig {
 public:
  FemPosConfig();
  ~FemPosConfig();
  int use_mode() const;
  double weight_fem_pos_deviation() const;
  double weight_ref_deviation() const;
  double weight_path_length() const;
  double weight_curvature_constraint_slack_var() const;
  double curvature_constraint() const;

  double sqp_ftol() const;
  double sqp_ctol() const;
  int sqp_pen_max_iter() const;
  int sqp_sub_max_iter() const;
  int max_iter() const;

  int print_level() const;
  int acceptable_num_of_iterations() const;
  double ipopt_tol() const;
  double ipopt_acceptable_tol() const;

  double time_limit() const;
  bool verbose() const;
  bool scaled_termination() const;
  bool warm_start() const;

  double longitudinal_boundary_bound() const;
  double lateral_boundary_bound() const;
  double reference_line_point_interval() const;

 private:
  int use_mode_ = 0;  // 0: osqp; 1: slack_osqp; 2: ipopt_slack
  double weight_fem_pos_deviation_ = 1.0e10;
  double weight_ref_deviation_ = 1.0;
  double weight_path_length_ = 1.0;
  double weight_curvature_constraint_slack_var_ = 1.0e2;
  double curvature_constraint_ = 0.2;

  double sqp_ftol_ = 1e-4;
  double sqp_ctol_ = 1e-3;
  int sqp_pen_max_iter_ = 10;
  int sqp_sub_max_iter_ = 100;
  // osqp settings
  int max_iter_ = 500;
  // ipopt setting
  int print_level_ = 1;
  int acceptable_num_of_iterations_ = 15;
  double ipopt_tol_ = 1.0e-8;
  double ipopt_acceptable_tol_ = 1.0e-1;
  // time_limit set to be 0.0 meaning no time limit
  double time_limit_ = 0.0;
  bool verbose_ = false;
  bool scaled_termination_ = true;
  bool warm_start_ = true;

  double longitudinal_boundary_bound_ = 1.0;
  double lateral_boundary_bound_ = 0.2;
  double reference_line_point_interval_ = 0.2;
};

}  // namespace planning
}  // namespace neodrive
