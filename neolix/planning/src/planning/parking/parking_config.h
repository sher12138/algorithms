#pragma once
#include <string>

namespace neodrive {
namespace planning {

struct ParkingCommonConfig {
  double delta_t_ = 0.5;
  double max_forward_v_ = 2.0;
  double max_reverse_v_ = 2.0;
  double max_forward_acc_ = 3.0;
  double max_reverse_acc_ = 2.0;
  double max_acc_jerk_ = 4.0;
  double path_resolution_ = 0.1;
  double park_spot_depth_buffer_ = 0.6;
  double path_time_resolution_ = 0.1;
  // front wheel angle, deg
  double max_steering_angle_ = 35.0;
};

struct ParkingFirstPlanConfig {
  // "hybrid a" or "geometric"
  std::string first_plan_type = "hybrid a";

  bool geo_use_smoothing = true;
  // geo_smoothing_iterative false--dis approach
  bool geo_smoothing_iterative = true;
};

struct ParkingCarportLimit {
  // perpendicular out
  double ppp_out_road_width = 4.8;
  double ppp_out_min_lateral_dis_to_road = 3.0;
};

struct ParkingHybridAStarConfig {
  // Hybrid a star for warm start
  double xy_grid_resolution_ = 0.3;
  double phi_grid_resolution_ = 0.1;
  double step_size_ = 0.5;
  int next_node_num_ = 10;
  double traj_forward_penalty_ = 2.0;
  double traj_back_penalty_ = 1.0;
  double traj_gear_switch_penalty_ = 2.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  // Grid a star for heuristic
  double grid_a_star_xy_resolution_ = 0.5;
  double node_radius_ = 0.25;
  double grid_sample_step_length_ = 1.0;
  bool verbose_ = false;  // used ?

  bool use_s_curve_speed_smooth__ = false;  // hybrida

  // reeds_shepp
  bool enable_parallel_hybrid_a_ = false;
  bool enable_optimal_path_selector_ = true;
  double rs_length_cost_ = 1.0;
  double rs_first_left_right_compliance_cost_ = 3.0;
  double rs_backward_length_cost_ = 2.0;
  double rs_first_backward_cost_ = 5.0;
  double rs_gear_switch_cost_ = 0.0;
};

struct ParkingTrajectoryOptimizerConfig {
  // true: use iterative, false: use distance approach
  bool use_iterative_anchoring_smoother_ = true;
  double is_near_destination_threshold_ = 0.05;
  // distance approach
  bool enable_smoother_failsafe_ = true;
  bool use_dual_variable_warm_start_ = true;
};

struct ParkingIterativeAnchoringConfig {
  // iterative_anchoring_smoother_config
  double interpolated_delta_s_ = 0.1;
  // flase use default, true based on obs
  bool estimate_bound_ = false;
  double default_bound_ = 2.0;
  double vehicle_shortest_dimension_ = 1.04;
  double collision_decrease_ratio_ = 0.9;
  int path_max_iteration_num_ = 50;
  // for reanchoring
  int reanchoring_trails_num_ = 50;
  double reanchoring_pos_stddev_ = 0.25;
  double reanchoring_length_stddev_ = 1.0;
  // for piecewise jerk speed
  double acc_weight_ = 1.0;
  double jerk_weight_ = 1.0;
  double kappa_penalty_weight_ = 100.0;
  double ref_s_weight_ = 10.0;
  double ref_v_weight_ = 0.0;
};

struct ParkingIpopt_config {
  int ipopt_print_level_ = 0;
  int mumps_mem_percent_ = 6000;
  double mumps_pivtol_ = 1.0e-6;
  int ipopt_max_iter_ = 100;
  double ipopt_tol_ = 1.0e-5;
  double ipopt_acceptable_constr_viol_tol_ = 0.1;
  double ipopt_min_hessian_perturbation_ = 1.0e-12;
  double ipopt_jacobian_regularization_value_ = 1.0e-7;
  std::string ipopt_print_timing_statistics_ = "yes";
  std::string ipopt_alpha_for_y_ = "min";
  std::string ipopt_recalc_y_ = "yes";
  double ipopt_mu_init_ = 0.1;
};

struct ParkingOsqp_config {
  bool debug_osqp_ = false;
  double beta_ = 1.0;
  // osqp setting
  double alpha_ = 1.0;  // Change alpha parameter
  double eps_abs_ = 1.0e-03;
  double eps_rel_ = 1.0e-03;
  int max_iter_ = 10000;
  bool polish_ = true;
  bool verbose_ = false;
};

struct ParkingDualVariableConfig {
  // DualVariableWarmStartConfig
  double weight_d_ = 1.0;

  int qp_format_ = 0;  // 0-osqp;1-slackqp; 2-ipoptqp;3-ipoptqp;
  double min_safety_distance_ = 0.01;

  ParkingOsqp_config osqp_config_;
  ParkingIpopt_config ipopt_config_;
};

struct DistanceApproachConfig {
  // Distance approach weight configs
  double weight_steer_ = 0.3;
  double weight_a_ = 1.1;
  double weight_steer_rate_ = 2.0;
  double weight_a_rate_ = 2.5;
  double weight_x_ = 18.0;
  double weight_y_ = 14.0;
  double weight_phi_ = 10.0;
  double weight_v_ = 0.0;
  double weight_steer_stitching_ = 1.75;
  double weight_a_stitching_ = 3.25;
  double weight_first_order_time_ = 1.0;
  double weight_second_order_time_ = 2.0;
  double min_safety_distance_ = 0.01;
  double min_time_sample_scaling_ = 0.5;
  double max_time_sample_scaling_ = 1.5;
  bool use_fix_time_ = false;
  ParkingIpopt_config ipopt_config_;
  bool enable_constraint_check_ = false;
  bool enable_initial_final_check_ = false;
  // 0-ipopt;1-ipopt_cuda; 2-ipopt-fixed_ts; 3-fixed_dual; 4-relax_end;
  // 5-relax_end_slack
  int distance_approach_mode_ = 0;
  bool enable_jacobian_ad_ = false;
  bool enable_check_initial_state_ = false;
  double weight_end_state_ = 1.0;
  double weight_slack_ = 1.0;
};

// TODO(wyc): no need to add getter
class ParkingConfig {
 public:
  ParkingConfig() {}
  ~ParkingConfig() {}

  ParkingCommonConfig common_config();
  ParkingFirstPlanConfig first_plan_park_in_config();
  ParkingFirstPlanConfig first_plan_park_out_config();
  ParkingHybridAStarConfig hybrid_a_star_config();
  ParkingTrajectoryOptimizerConfig park_trajectory_optimizer_config();
  ParkingIterativeAnchoringConfig iterative_anchor_config();

  ParkingDualVariableConfig dual_variable_config();
  DistanceApproachConfig distacne_approach_config();

 private:
  // TODO(wyc): name
  bool FraseFromConfig();

  ParkingCommonConfig common_config_;
  ParkingFirstPlanConfig first_plan_park_in_config_;
  ParkingFirstPlanConfig first_plan_park_out_config_;
  ParkingHybridAStarConfig hybrid_a_star_config_;
  ParkingTrajectoryOptimizerConfig park_trajectory_optimizer_config_;
  ParkingIterativeAnchoringConfig iterative_anchor_config_;

  ParkingDualVariableConfig dual_variable_config_;
  DistanceApproachConfig distacne_approach_config_;
};

}  // namespace planning
}  // namespace neodrive
