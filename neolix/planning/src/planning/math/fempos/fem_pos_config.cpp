#include "fem_pos_config.h"

namespace neodrive {
namespace planning {

FemPosConfig::FemPosConfig() {}
FemPosConfig::~FemPosConfig() {}
int FemPosConfig::use_mode() const { return use_mode_; }
double FemPosConfig::weight_fem_pos_deviation() const {
  return weight_fem_pos_deviation_;
}
double FemPosConfig::weight_ref_deviation() const {
  return weight_ref_deviation_;
}
double FemPosConfig::weight_path_length() const { return weight_path_length_; }

double FemPosConfig::weight_curvature_constraint_slack_var() const {
  return weight_curvature_constraint_slack_var_;
}
double FemPosConfig::curvature_constraint() const {
  return curvature_constraint_;
}

double FemPosConfig::sqp_ftol() const { return sqp_ftol_; }
double FemPosConfig::sqp_ctol() const { return sqp_ctol_; }
int FemPosConfig::sqp_pen_max_iter() const { return sqp_pen_max_iter_; }
int FemPosConfig::sqp_sub_max_iter() const { return sqp_sub_max_iter_; }

int FemPosConfig::max_iter() const { return max_iter_; }
int FemPosConfig::print_level() const { return print_level_; }
int FemPosConfig::acceptable_num_of_iterations() const {
  return acceptable_num_of_iterations_;
}
double FemPosConfig::ipopt_tol() const { return ipopt_tol_; }
double FemPosConfig::ipopt_acceptable_tol() const {
  return ipopt_acceptable_tol_;
}
double FemPosConfig::time_limit() const { return time_limit_; }
bool FemPosConfig::verbose() const { return verbose_; }
bool FemPosConfig::scaled_termination() const { return scaled_termination_; }
bool FemPosConfig::warm_start() const { return warm_start_; }
double FemPosConfig::longitudinal_boundary_bound() const {
  return longitudinal_boundary_bound_;
}
double FemPosConfig::lateral_boundary_bound() const {
  return lateral_boundary_bound_;
}
double FemPosConfig::reference_line_point_interval() const {
  return reference_line_point_interval_;
}

}  // namespace planning
}  // namespace neodrive
