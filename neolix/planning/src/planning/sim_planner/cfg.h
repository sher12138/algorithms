#pragma once
namespace neodrive {
namespace planning {
namespace sim_planner {
struct Effciency {
  double ego_lack_speed_to_desired_unit_cost = 0.3;
  double ego_over_speed_to_desired_unit_cost = 0.03;
  double ego_desired_speed_tolerate_gap = 1.0;
  double leading_distance_th = 120.0;
  double min_distance_ratio = 0.4;
  double ego_speed_blocked_by_leading_unit_cost = 0.02;
  double leading_speed_blocked_desired_vel_unit_cost = 0.1;
};

// struct Safety {
//   bool rss_cost_enable = true;
//   double rss_over_speed_power_coeff = 0.25;
//   double rss_over_speed_linear_coeff = 0.5;
//   double rss_lack_speed_power_coeff = 0.35;
//   double rss_lack_speed_linear_coeff = 0.6;
//   bool occu_lane_enable = true;
//   double occu_lane_unit_cost = 0.10;
// };

struct User {
  double late_operate_unit_cost = 0.25;
  double cancel_operation_unit_cost = 8.0;  // cancel cost
};

struct Navigation {
  double lane_change_left_unit_cost = 0.015;
  double lane_change_right_unit_cost = 0.06;
  double lane_change_unit_cost_vel_lb = 10.0;
  double lane_change_left_recommendation_reward = 1.0;
  double lane_change_right_recommendation_reward = 1.0;
};

struct Offsetdriving {
  double lane_center_driving_coeff = 2.8;
};

struct Cost {
  Effciency effciency_;
  // safety safety_;
  User user_;
  Navigation navigation_;
  Offsetdriving offsetdriving_;
  double discount_factor = 1.0;
};

struct Duration {
  double layer = 1.0;
  double last_layer = 1.0;
  double step = 0.2;
  int tree_height = 5;
};

struct EgoLonIDM {
  double min_spacing = 2.0;
  double head_time = 1.0;
  double exponent = 4;
};

struct EgoLonLimit {
  double acc = 1.0;
  double acc_jerk = 3.0;
  double soft_brake = 1.67;
  double hard_brake = 5.0;
  double brake_jerk = 5.0;
};

struct EgoLatPurePursuit {
  double gain = 2.5;
  double max_lookahead_dist = 100.0;
  double min_lookahead_dist = 0.2;
};

struct EgoLatLimit {
  double acc = 1.2;
  double jerk = 1.8;
  double curvature = 0.33;
  double steer_angle = 0.785;
  double steer_rate = 0.39;
};

// struct Refline {
//   double len_vel_coeff = 10.0;
//   double forward_len_max = 150.0;
//   double forward_len_min = 50.0;
//   double backward_len_max = 20.0;
// };

struct EgoLon {
  EgoLonIDM ego_lon_idm_;
  EgoLonLimit ego_lon_limit_;
};

struct EgoLat {
  EgoLatPurePursuit ego_lat_pure_pursuit_;
  EgoLatLimit ego_lat_limit_;
};

struct Ego {
  EgoLon ego_lon_;
  EgoLat ego_lat_;
  bool auto_dec_if_lat_failed = true;
  double cooperative_lat_range = 1.1;
  double lon_aggressive_ratio = 0.25;
};

struct Sim {
  Duration duration_;
  Ego ego_;
  double acc_cmd_vel_gap = 10.0;
  double dec_cmd_vel_gap = 10.0;
};

struct ActiveLC {
  double cold_duration = 0.0;
  double activate_speed_lower_bound = 1.0;
  double activate_speed_upper_bound = 24.0;
  double activate_max_duration_in_seconds = 3.8;
  double active_min_operation_in_seconds = 2.0;
  double consistent_operate_time_min_gap = 0.5;
  double consistent_min_num_frame = 4;
  bool enable_clear_accumulation_by_forbid_signal = true;
  bool enable_auto_cancel_by_forbid_signal = false;
  bool enable_auto_cancel_by_outdate_time = true;
  bool enable_auto_canbel_by_stick_signal = true;
  double auto_cancel_if_late_for_seconds = 15.0;
};

struct Function {
  bool mobil_enable = false;
  bool active_lc_enable = true;
  ActiveLC active_lc_;
  double stick_lane_change_in_seconds = 0.0;
};

// struct Strict {
//   double inflation_w = 0.1;
//   double inflation_h = 0.1;
// };

struct Safety {
  bool strict_check_enable = true;
  // Strict strict_;
};

struct Cfg {
  Cost cost_;
  Sim sim_;
  Function function_;
  Safety Safety_;
};
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive