/**
 * @brief this file is auto-generated by scripts/json_to_cpp.py. Do not edit!!!
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace neodrive {
namespace planning {
namespace config {

struct AutoPlanConfig {
  std::string planning_node_name;
  std::string motion_task_name;
  struct Common {
    std::string planning_data_path;
    std::string planning_scene_special_configure_file;
    float path_resolution;
    float path_parking_resolution;
    float total_time;
    float delta_t;
    bool use_ref_line_limit;
    float junction_max_speed;
    float crosswalk_max_speed;
    float max_av_plus;
    float max_av_minus;
    float max_jerk_plus;
    float max_jerk_minus;
    float centric_accel_limit;
    float spot_stop_speed;
    float parking_speed;
    float reverse_speed;
    float static_hard_distance;
    float dynamic_hard_distance;
    float dynamic_nudge_speed_percent;
    float static_nudge_max_speed;
    float slope_consider_angle_degree;
    bool add_reinforcement_learning_event;
    float closer_obs_distance_threshold;
    float little_obs_area_threshold;
    float dynamic_obs_speed_threshold;
    float bias_to_right_lane_bound;
    float bias_to_right_obs_bound;
    float motorway_bias_to_obs_bound;
    float motorway_bias_to_unmovable_obs_bound;
    float bias_to_right_in_turn;
    bool only_for_single_cipv;
    bool enable_camera_lines;
    bool enable_camera_lines_in_junction;
    bool enable_back_cipv;
    float destination_virtual_obs_buffer;
    bool use_center_pose;
  };
  Common common;
  struct Map {
    std::string planning_base_map_path;
    std::string planning_map_dir;
  };
  Map map;
  struct SpeedPlan {
    bool check_collision_with_prediction_trajectory;
    float predict_trajectory_time;
    float max_predict_trajectory_dis;
    float speed_delta_and_headingdiff_ratio;
    float filter_dynamic_obs_safe_distance_ratio;
    float turn_left_min_kappa;
    int turn_left_check_sector_radius;
    float left_turn_front_extend;
    float left_turn_back_extend;
    float left_turn_min_len_threshold;
    float dynamic_obs_area_min_length;
    float dynamic_obs_area_min_time;
    float uniform_trajectory_attention_lon_time;
    float uniform_trajectory_attention_lat_time;
    float turn_right_max_kappa;
    int turn_right_check_sector_radius;
    float right_turn_front_extend;
    float right_turn_back_extend;
    float right_turn_min_len_threshold;
    float judge_turn_min_time;
    float judge_turn_min_length;
    float judge_turn_min_kappa;
    int judge_turn_continuous_point_num;
  };
  SpeedPlan speed_plan;
  struct InlaneUturn {
    struct InlaneUturnCommon {
      float delta_t;
      float max_forward_v;
      float max_reverse_v;
      float max_forward_acc;
      float max_reverse_acc;
      float max_acc_jerk;
      float path_resolution;
      float park_spot_depth_buffer;
      float path_time_resolution;
      float max_steering_angle;
    };
    InlaneUturnCommon inlane_uturn_common;
    struct InlaneUturnHybridA {
      float xy_grid_resolution;
      float phi_grid_resolution;
      float step_size;
      int next_node_num;
      float traj_forward_penalty;
      float traj_back_penalty;
      float traj_gear_switch_penalty;
      float traj_steer_penalty;
      float traj_steer_change_penalty;
      float grid_a_star_xy_resolution;
      float node_radius;
      bool verbose;
      bool use_s_curve_speed_smooth;
      bool enable_parallel_hybrid_a;
      bool enable_optimal_path_selector;
      float rs_length_cost;
      float rs_first_left_right_compliance_cost;
      float rs_backward_length_cost;
      float rs_first_backward_cost;
      float rs_gear_switch_cost;
    };
    InlaneUturnHybridA inlane_uturn_hybrid_a;
  };
  InlaneUturn inlane_uturn;
  struct OutlaneUturn {
    struct OutlaneUturnCommon {
      float delta_t;
      float max_forward_v;
      float max_reverse_v;
      float max_forward_acc;
      float max_reverse_acc;
      float max_acc_jerk;
      float path_resolution;
      float park_spot_depth_buffer;
      float path_time_resolution;
      float max_steering_angle;
    };
    OutlaneUturnCommon outlane_uturn_common;
    struct OutlaneUturnHybridA {
      float xy_grid_resolution;
      float phi_grid_resolution;
      float step_size;
      int next_node_num;
      float traj_forward_penalty;
      float traj_back_penalty;
      float traj_gear_switch_penalty;
      float traj_steer_penalty;
      float traj_steer_change_penalty;
      float grid_a_star_xy_resolution;
      float node_radius;
      bool verbose;
      bool use_s_curve_speed_smooth;
      bool enable_parallel_hybrid_a;
      bool enable_optimal_path_selector;
      float rs_length_cost;
      float rs_first_left_right_compliance_cost;
      float rs_backward_length_cost;
      float rs_first_backward_cost;
      float rs_gear_switch_cost;
    };
    OutlaneUturnHybridA outlane_uturn_hybrid_a;
  };
  OutlaneUturn outlane_uturn;
  struct PilotState {
    float alarm_escalation_interval;
    float degradation_speed_limit;
    float system_start_time;
    float frequent_degradation_interval;
    float normal_upgradation_interval;
    float escalation_hold_interval;
    float smooth_brake_brake_speed;
    float smooth_brake_speed_limit_step;
    float lidar_perception_error_threshold;
    float odom_sensor_delay_threshold;
    float utm_sensor_delay_threshold;
    float localization_msg_delay_threshold;
    float perception_msg_delay_threshold;
    float localization_position_jump_threshold;
    int lidar_freespace_empty_threshold;
    float lidar_non_point_low_speed_limit;
    float lidar_non_point_medium_speed_limit;
    float lidar_non_point_high_speed_limit;
    int odom_wheel_error_cnt_threshold;
  };
  PilotState pilot_state;
  struct NarrowRoadScenario {
    float narrow_road_width_threshold;
    float exit_narrow_road_width_buffer;
    float preview_distance;
  };
  NarrowRoadScenario narrow_road_scenario;
  struct IntersectionScenario {
    float approach_distance_threshold;
    float approach_time_threshold;
    float leave_distance_threshold;
    float traffic_light_distance_threshold;
    int detect_length;
    float kappa_threshold;
  };
  IntersectionScenario intersection_scenario;
  struct SideWayIntersectionScenario {
    bool enable_side_way_intersection;
    float crossroad_preview_distance;
    float traffic_light_preview_distance;
  };
  SideWayIntersectionScenario side_way_intersection_scenario;
  struct CruiseScenario {
    bool enable_bias_drive;
    bool enable_left_overtake;
    bool enable_right_avoid;
    float right_road_buff;
    float right_avoid_buff;
    float left_road_buff;
    float left_overtake_buff;
    int filter_frame_threshold;
  };
  CruiseScenario cruise_scenario;
  struct MotorwayCruiseScenario {
    int driving_direction;
    int filter_frame_threshold;
  };
  MotorwayCruiseScenario motorway_cruise_scenario;
  struct MotorwayDetourScenario {
    int driving_direction;
    float preview_front_distance;
    float preview_back_distance;
    float near_front_distance;
    float near_back_distance;
    float preview_time;
    float preview_distance;
    float adjacent_lanes_preview_distance;
    float left_min_lane_borrow_dis;
    float left_max_lane_borrow_dis;
    float right_min_lane_borrow_dis;
    float right_max_lane_borrow_dis;
    float right_first_line_preview_distance;
    float filter_obs_heading_threshold;
    float detour_triggered_space;
    bool queued_crossroad_check;
    float preview_road_bound_close_ignore_distance;
    float preview_crossroad_close_ignore_distance;
    float preview_lane_turn_close_ignore_distance;
    struct DetourEnterScenario {
      int crossroad_preview_distance;
      int queued_crossroad_preview_distance;
      int traffic_light_preview_distance;
      int queued_traffic_light_preview_distance;
      int road_bound_preview_distance;
      int lane_turn_preview_distance;
    };
    DetourEnterScenario detour_enter_scenario;
    struct DetourExitScenario {
      int crossroad_preview_distance;
      int queued_crossroad_preview_distance;
      int traffic_light_preview_distance;
      int queued_traffic_light_preview_distance;
      int road_bound_preview_distance;
      int lane_turn_preview_distance;
    };
    DetourExitScenario detour_exit_scenario;
  };
  MotorwayDetourScenario motorway_detour_scenario;
  struct MotorwayIntersectionScenario {
    float signal_detect_distance;
    float lanetype_detect_distance;
  };
  MotorwayIntersectionScenario motorway_intersection_scenario;
  struct DetourScenario {
    int driving_direction;
    float preview_front_distance;
    float preview_back_distance;
    float near_front_distance;
    float near_back_distance;
    float preview_time;
    float preview_distance;
    float left_min_lane_borrow_dis;
    float left_max_lane_borrow_dis;
    float right_min_lane_borrow_dis;
    float right_max_lane_borrow_dis;
    float detour_triggered_space;
    float filter_obs_heading_threshold;
    bool queued_crossroad_check;
    float preview_road_bound_close_ignore_distance;
    float preview_crossroad_close_ignore_distance;
    struct DetourEnterScenario {
      int crossroad_preview_distance;
      int queued_crossroad_preview_distance;
      int traffic_light_preview_distance;
      int queued_traffic_light_preview_distance;
      int road_bound_preview_distance;
    };
    DetourEnterScenario detour_enter_scenario;
    struct DetourExitScenario {
      int crossroad_preview_distance;
      int queued_crossroad_preview_distance;
      int traffic_light_preview_distance;
      int queued_traffic_light_preview_distance;
      int road_bound_preview_distance;
    };
    DetourExitScenario detour_exit_scenario;
  };
  DetourScenario detour_scenario;
  struct Parking {
    struct VerticalParkingSpace {
      float front_dis_in_park;
      float c_type_front_dis_in_park;
      float boundary_dist;
      float soft_radius;
      float soft_bios;
      float prepare_radius;
      float dist_to_end;
      float park_out_soft_radius;
      float c_type_dis_threshold;
      float c_type_radius;
      float c_type_dist_to_end;
      float prediction_dis;
      float lateral_buffer;
      float l_type_point_c_dis;
      float l_type_dist_to_end;
      float park_out_extend_length;
      float theta;
      float parking_space_lateral_buffer;
      float parking_space_vertical_buffer;
      float min_speed;
      float degree_buffer;
    };
    VerticalParkingSpace vertical_parking_space;
    struct HorizontalParkingSpace {
      float dist_to_end;
      float front_safe_dist;
      float back_safe_dist;
      float front_offset;
      float back_offset;
      float finish_heading_threshold;
      float parking_space_vertical_buffer;
      float forward_max_speed;
      float backward_max_speed;
      float lane_ignore_dis_threshold;
    };
    HorizontalParkingSpace horizontal_parking_space;
    struct OnlaneParkingSpace {
      float dis_to_end;
      float extern_dis;
    };
    OnlaneParkingSpace onlane_parking_space;
    float parking_in_max_speed;
    float parking_out_max_speed;
    float parking_out_kappa_threshold;
    float parkint_out_default_speed;
    float parking_out_speed_in_turn;
    float prediction_dis;
    float lateral_buffer;
    float preview_dis;
    bool path_detect;
    bool steering_detect;
    bool heading_detect;
    float heading_detect_pre_dis;
    float default_speed;
    float heading_diff_threshold;
    float high_dis_error_threshold;
    float low_dis_error_threshold;
    float high_dis_error_speed;
    float low_dis_error_speed;
    float parking_in_detect_max_dis;
    float forward_max_speed;
    float backward_max_speed;
    float max_jerk;
    float dist_slow_down_coef;
    float heading_slow_down_coef;
    float parking_slow_down_dist;
    float stop_time_threhold;
    float parking_stuck_speed_threshold;
    float camera_obs_filter_dis;
    bool enable_camera_freespace;
  };
  Parking parking;
  struct Indoor {
    float lidar_freespace_max_distance;
    float freespace_filter_length_threshold;
    float freespace_filter_speed_threshold;
    float max_speed;
    float low_speed;
    float min_acc;
    float max_acc;
    float freespace_low_speed_dis_threshold;
    float freespace_stop_dis_threshold;
    float max_jerk;
    float centric_accel_limit;
    bool enable_obstacle;
    float radius;
    int steer_threshold;
  };
  Indoor indoor;
  struct RecordEvent {
    float emergency_brake_accel_threshold;
    float into_degradation_before_time_s;
    float into_degradation_after_time_s;
    float into_escalation_before_time_s;
    float into_escalation_after_time_s;
    float take_over_before_time_s;
    float take_over_after_time_s;
    float emergency_brake_before_time_s;
    float emergency_brake_after_time_s;
  };
  RecordEvent record_event;
  struct PullOver {
    float early_check_queue_distance;
    float check_obstacle_distance;
    float stop_queue_vehicle_distance;
    float wujiang_stop_queue_vehicle_distance;
    float check_stop_distance_to_boundary;
    float back_close_obs_distance;
    float allow_pull_over_max_speed;
    float station_stop_deceleration_ratio;
  };
  PullOver pull_over;
  struct BiasDriving {
    bool enable_road_has_bound_bias;
    float road_has_bound_residual;
    float road_bound_preview;
    float road_has_bound_max_width;
    float front_junction_close_distance;
    float junction_concern_distance;
    float lane_turn_preview_distance;
    float lane_turn_kappa_threshold;
    float min_shift_distance;
    bool enable_board_road_bias;
    float board_road_width_into_threshold;
    float board_road_width_exit_threshold;
    float board_road_bias_residual;
    float side_far_obs_distance;
    float same_direction_heading_threshold;
    float slow_obs_velocity_threshold;
    float preview_back_close_distance;
    float preview_back_far_distance;
    bool trigger_stop_need_pull_over;
  };
  BiasDriving bias_driving;
  struct AbnormalWatcher {
    float lateral_attention_ratio;
  };
  AbnormalWatcher abnormal_watcher;
  struct TrafficLaw {
    float stop_signal_preview_distance;
    float y_junction_stop_interval;
    float length_of_passing_stop_line_buffer;
    float crosswalk_min_speed;
    float crosswalk_lateral_min_dis;
    float crosswalk_longitudinal_max_dis;
    float passing_speed_threshold;
    float passing_speed_ratio;
    float passing_time_left_time_threshold;
    float passing_time_max_threshold;
    float junction_length_threshold;
    float junction_export_finished_threshold;
    float max_deceleration;
    float max_over_line_distance;
    float max_brake_over_line_distance;
    float color_confirm_time_threshold;
    float green_light_pass_max_distance;
    float back_mask_heading;
    float stop_line_distance;
    float stop_line_speed;
    float stop_compensation_distance;
  };
  TrafficLaw traffic_law;
  struct TrafficLightForPerception {
    float front_attention_dis;
    float back_attention_dis;
    float forward_junction_dis;
  };
  TrafficLightForPerception traffic_light_for_perception;
  struct Intention {
    float turn_intention_preview_distance;
    float turn_intention_kappa_threshold;
  };
  Intention intention;
  struct HumanInterface {
    float block_time_ask_for_takeover;
    float ask_for_take_over_last_time;
    float request_yield_length;
    float slow_speed_threshold;
    float no_move_dist_threshold;
    float no_obstacle_stop_time_threshold;
    float obstacle_block_stop_time_threshold;
    int reach_station_keep_cnt;
    float station_stop_speed_threshold;
    float reach_station_hard_dist_threashold;
    float reach_station_soft_dist_threashold;
    float reach_station_hard_stop_time;
    float reach_station_soft_stop_time;
    float no_abnormal_stop_check_dist;
  };
  HumanInterface human_interface;
  struct SpeedLimit {
    bool enable_start_up_limit;
    float start_up_limit;
    float prepare_borrowing_limit;
    float lane_borrowing_limit;
    float multi_level_strong_passing;
    float multi_level_passing_through;
    float multi_level_passing_by;
    float multi_level_normal;
    float multi_level_high_speed;
    float close_to_station;
    bool enable_moving_obs_dis_limit;
    bool enable_road_bound_limit;
    float static_obs_speed_limit_safe_dis;
    float speed_limit_slow_down_accel;
    float kappa_speed_limit_dis;
    float max_check_kappa_len;
    float dynamic_speedlimit_ignore_dist;
    float dynamic_speedlimit_consider_length;
    float request_yield_ignore_dist;
    float accessable_path_attention_length;
    float accessable_path_attention_time;
  };
  SpeedLimit speed_limit;
  struct SpeedReversedObs {
    float reverse_heading_diff_threshold_rad;
    float emergency_deal_time;
    float emergency_deal_min_s_len;
    float emergency_speed_limit_ratio;
    float slow_down_deal_time;
    float slow_down_deal_min_s_len;
    bool is_check_before_come_in_single_road;
    float single_road_stop_wait_buffer;
    float all_delay_time;
    float lateral_safe_dis;
    float lateral_cutin_delta;
    int stop_cutin_filter_num;
    int linear_ratio_v_and_l_dis;
    float linear_offset_v_and_l_dis;
    float limit_max_deceleration;
    float lat_check_time;
    float lat_check_dis;
  };
  SpeedReversedObs speed_reversed_obs;
  struct SpeedWormOrSpeedUp {
    float worm_check_area_width_epsilon;
    float worm_check_area_length;
    float worm_adc_speed_threshold;
    int worm_adc_time_cycle_threshold;
    float worm_min_safe_distance;
    float worm_obs_max_speed_threshold;
    float speed_up_check_area_min_length;
    float speed_up_safe_distance;
    bool speed_up_consider_obs;
    float speed_up_safe_speed_threshold;
    float speed_up_max_deceleration_threshold;
  };
  SpeedWormOrSpeedUp speed_worm_or_speed_up;
  struct SpeedSmoothSlowDown {
    float static_obs_slow_down_buffer;
    float virtual_obs_slow_down_buffer;
    float slow_down_target_speed;
    float all_delay_time;
    float limit_max_deceleration;
  };
  SpeedSmoothSlowDown speed_smooth_slow_down;
  struct SpeedStaticObsJump {
    float adc_enlarge_buffer;
    float static_obs_slow_down_buffer;
    float lat_dis_speed_ratio;
  };
  SpeedStaticObsJump speed_static_obs_jump;
  struct VisualRefLine {
    bool force_enter_visual_ref_line;
    bool enable_visual_ref_line;
  };
  VisualRefLine visual_ref_line;
  struct BackOut {
    bool enable_back_out;
    bool enable_motor_back_out;
    float end_length;
    float tar_s_bias;
    bool detour_choose;
    float max_limit_speed;
    float lat_err;
    float station_err;
    float heading_err;
    std::string camera_id;
    int stop_cnt_threshold;
    int preview_distance;
    float lat_safe_buffer;
    float lon_safe_buffer;
  };
  BackOut back_out;
  struct EventReport {
    bool enable_preprocess_fail;
    float preprocess_fail_mintime;
    bool enable_plan_fail;
    float plan_fail_mintime;
    bool enable_unknown_traffic_light;
    float unknown_traffic_light_mintime;
    bool enable_stop_over_line;
    float stop_over_line_mintime;
    bool enable_pullover_failed;
    float pullover_failed_mintime;
    bool enable_meet_barrier_gate;
    float meet_barrier_gate_mintime;
    bool enable_parkin_finish;
    float parkin_finish_mintime;
    bool enable_parkout_finish;
    float parkout_finish_mintime;
    bool enable_park_failed;
    float park_failed_mintime;
    bool enable_abnormal_stop;
    float abnormal_stop_mintime;
    bool enable_obstacle_block_timeout;
    float obstacle_block_timeout_mintime;
    bool enable_detour_failed;
    float detour_failed_mintime;
    bool enable_lane_change_failed;
    float lane_change_failed_mintime;
    bool enable_mixed_lane_change_failed;
    float mixed_lane_change_failed_mintime;
  };
  EventReport event_report;
  struct Localization {
    float odom_keep_dis;
    float hdmap_utm_delay_threshold;
    float hdmap_odom_delay_threshold;
    float odom_sensor_bad_deceleration;
    float odom_sensor_bad_system_delay_time;
  };
  Localization localization;
  struct Prediction {
    bool enable_prediction;
    float reverse_trust_time_delta;
    float trust_time_delta;
    bool enable_meeting_turn_right;
  };
  Prediction prediction;
  struct BarrierGateScenario {
    bool enable_barrier_gate;
    float move_stage_speed_limit;
    float virtual_obs_buffer_dis;
    bool flag_complete_auto_drive;
    bool flag_test;
  };
  BarrierGateScenario barrier_gate_scenario;
  struct MotorwayLaneChangeScenario {
    float preview_front_distance;
    float preview_back_distance;
    float near_front_distance;
    float near_back_distance;
    float preview_time;
    float traffic_light_preview_distance;
    float road_bound_preview_distance;
    float divider_restrict_preview_distance;
    float lane_turn_preview_distance;
    int motorway_lane_change_request;
    float min_stop_dist_before_junction;
    float min_distance_to_lane_change_end;
    float adc_width_ratio;
    float buff_of_maxspeed;
    float buff_of_minspeed;
    float maxspeed;
    float minspeed;
    float max_deceleration;
  };
  MotorwayLaneChangeScenario motorway_lane_change_scenario;
  struct ReverseLaneDetour {
    bool enable_reverse_lane_detour_motorway;
    bool enable_reverse_lane_detour_non_motorway;
    struct SpeedLimitMotorway {
      float leftmost_lane_complete_limit;
      float divider_crossing_limit;
      float reverse_lane_complete_limit;
      float over_reverse_lane_limit;
      float not_define_limit;
    };
    SpeedLimitMotorway speed_limit_motorway;
    struct SpeedLimitNonMotorway {
      float leftmost_lane_complete_limit;
      float divider_crossing_limit;
      float reverse_lane_complete_limit;
      float over_reverse_lane_limit;
      float not_define_limit;
    };
    SpeedLimitNonMotorway speed_limit_non_motorway;
  };
  ReverseLaneDetour reverse_lane_detour;
  struct ScenarioCommon {
    struct IsFrontStaticObsClearConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsFrontStaticObsClearConf is_front_static_obs_clear_conf;
    struct IsDynamicObsClearConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsDynamicObsClearConf is_dynamic_obs_clear_conf;
    struct IsFrontHasTrafficLightConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsFrontHasTrafficLightConf is_front_has_traffic_light_conf;
    struct IsFrontHasCrossRoadConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsFrontHasCrossRoadConf is_front_has_cross_road_conf;
    struct IsFrontHasRoadBoundaryConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsFrontHasRoadBoundaryConf is_front_has_road_boundary_conf;
    struct IsLeftFrontHasRoadBoundaryConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsLeftFrontHasRoadBoundaryConf is_left_front_has_road_boundary_conf;
    struct IsRightFrontHasRoadBoundaryConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsRightFrontHasRoadBoundaryConf is_right_front_has_road_boundary_conf;
    struct IsInRightFirstLineConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsInRightFirstLineConf is_in_right_first_line_conf;
    struct IsFrontHasLaneTurnConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsFrontHasLaneTurnConf is_front_has_lane_turn_conf;
    struct IsAllowedDetourInReverseLaneConf {
      bool is_use_multi_frames;
      int horizon;
      float exp_param;
    };
    IsAllowedDetourInReverseLaneConf is_allowed_detour_in_reverse_lane_conf;
  };
  ScenarioCommon scenario_common;
};

void InitAutoPlanConfig(const Json::Value &input_json, AutoPlanConfig &dest);

}  // namespace config
}  // namespace planning
}  // namespace neodrive
