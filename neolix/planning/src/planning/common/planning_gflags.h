#pragma once

#include <math.h>

#include "src/planning/public/planning_lib_header.h"

//--------common configure--------
//--------PlanningManager--------
DECLARE_int32(planning_max_history_result);
DECLARE_int32(planning_period_frequence);
DECLARE_double(planning_still_obstacle_speed_threshold);
DECLARE_double(planning_still_pedestrian_speed_threshold);

//--------finite state machine--------
DECLARE_int32(planning_state_fail_threshold);

//--------record sim------
DECLARE_bool(playing_record);
DECLARE_bool(planning_piecewise_use_four_circles_model);

//--------spend time record-----------
DECLARE_bool(enable_record_spend_time);

//--------channels--------
DECLARE_string(planning_localization_reader_config_file);

DECLARE_bool(planning_enable_publish_odometry_debug_flag);

// planning test channels
DECLARE_bool(planning_test_path_publish);
DECLARE_bool(planning_test_speed_publish);
DECLARE_bool(planning_enable_vis_event);

//--------worldwide flag--------
DECLARE_bool(planning_default_left_right_side);

//--------obstacle macro define--------
DECLARE_int32(planning_object_table_obstacle_capacity);
DECLARE_double(planning_polygon_length_box_length_max_diff);
DECLARE_int32(planning_obstacle_record_history_frame_cnt);
DECLARE_bool(planning_obstacle_enable_time_diff);
DECLARE_double(planning_obstacle_max_time_diff);

// virtual obstacle
DECLARE_double(planning_virtual_obstacle_length);
DECLARE_double(planning_virtual_obstacle_max_width);
DECLARE_double(planning_virtual_obstacle_min_width);
DECLARE_double(planning_virtual_obstacle_height);

//--------trajectory stitching--------
DECLARE_bool(planning_use_stitch);
DECLARE_bool(planning_use_stitch_with_async);
DECLARE_bool(planning_enable_time_stitch);
DECLARE_double(planning_path_predict_time);
DECLARE_double(planning_speed_predict_time);
DECLARE_double(planning_forward_predict_time);
DECLARE_double(planning_last_trajectory_stub_time);
DECLARE_double(planning_replan_distance_threshold);
DECLARE_double(planning_replan_heading_threshold);
DECLARE_int32(planning_stitch_tracking_buffer_size);
DECLARE_double(planning_stitch_moving_distance_threshold);
DECLARE_bool(planning_use_hermite_spline_interpolate);

// control feedback trigger replan
DECLARE_bool(use_decouple);

//--------reference line--------
DECLARE_double(planning_new_routing_start_s_max_diff);
DECLARE_double(planning_reference_line_extend_length);
DECLARE_double(default_reference_line_width);
DECLARE_double(invalid_reference_line_heading_diff_threshold);
DECLARE_double(routing_point_gap);
DECLARE_double(reference_line_start_s);
DECLARE_double(reference_line_end_s);
DECLARE_double(reference_line_dis_threshold);
DECLARE_bool(reference_line_enable_precise_road_bound_calculation);

// refer line in decision
DECLARE_double(planning_look_forward_short_distance);
DECLARE_double(planning_look_forward_long_distance);
DECLARE_double(planning_look_backward_distance);

//--------reverse driving--------
DECLARE_bool(planning_enable_reverse_driving);
DECLARE_int32(planning_reverse_driving_trigger_conts);
DECLARE_double(planning_reverse_driving_trigger_probability);
DECLARE_double(planning_reverse_driving_front_l_range);
DECLARE_double(planning_reverse_driving_front_s_range);
DECLARE_double(planning_reverse_driving_back_lat_thresh);
DECLARE_bool(planning_enable_reverse_tuining_control);
// reverse line function
DECLARE_double(planning_reverse_driving_max_distance);
DECLARE_double(planning_reverse_driving_finish_distance_threshold);

// narrow mode
DECLARE_bool(planning_use_narrow_crossing);

//--------inlane uturn--------
DECLARE_bool(planning_enable_inlane_uturn);
DECLARE_bool(planning_piecewise_use_soft_constraints);
DECLARE_bool(planning_inlane_uturn_ignore_obs);
DECLARE_double(planning_inlane_uturn_adjust_length);
DECLARE_double(planning_inlane_uturn_front_buffer);
DECLARE_double(planning_inlane_uturn_rear_buffer);
DECLARE_double(planning_inlane_uturn_left_buffer);
DECLARE_double(planning_inlane_uturn_right_buffer);
DECLARE_int32(planning_inlane_uturn_trigger_conts);
DECLARE_double(planning_inlane_uturn_trigger_probability);

// inlane uturn refer line
DECLARE_double(planning_inlane_uturn_min_width);
DECLARE_double(planning_inlane_uturn_backward_refer_line_s);
DECLARE_double(planning_inlane_uturn_forward_refer_line_s);
DECLARE_double(planning_inlane_uturn_reach_distance_threshold);
DECLARE_double(planning_inlane_uturn_reach_heading_threshold);
DECLARE_double(planning_inlane_uturn_min_radius);
DECLARE_double(planning_inlane_uturn_shrink_border_dis);
DECLARE_bool(planning_enable_inlane_uturn_obs_trigger);
DECLARE_double(planning_inlane_uturn_front_clear_s);
DECLARE_double(planning_inlane_uturn_rear_clear_s);
DECLARE_double(planning_inlane_uturn_left_clear_l);
DECLARE_double(planning_inlane_uturn_right_clear_l);

//--------out lane u-turn--------
DECLARE_bool(planning_enable_outlane_uturn);
DECLARE_double(planning_outlane_uturn_trigger_radius);
DECLARE_int32(planning_outlane_uturn_trigger_conts);
DECLARE_double(planning_outlane_uturn_trigger_probability);
DECLARE_double(planning_outlane_uturn_backward_refer_line_s);
DECLARE_double(planning_outlane_uturn_forward_refer_line_s);
DECLARE_double(planning_outlane_uturn_reach_distance_threshold);
DECLARE_double(planning_outlane_uturn_reach_heading_threshold);

//--------self adjust function--------
DECLARE_bool(planning_self_adjust_enable_flag);
DECLARE_double(planning_self_adjust_trigger_min_heading_diff);
DECLARE_double(planning_self_adjust_trigger_max_heading_diff);
DECLARE_bool(planning_self_adjust_enable_cross_road_bound_flag);
DECLARE_double(planning_self_adjust_cross_road_bound_buffer);
DECLARE_double(planning_self_adjust_forward_clear_zone_s);
DECLARE_double(planning_self_adjust_backward_clear_zone_s);
DECLARE_double(planning_self_adjust_lateral_clear_zone_l);
DECLARE_int32(planning_self_adjust_trigger_conts);
DECLARE_double(planning_self_adjust_trigger_probability);
// self adjust line function
DECLARE_double(planning_self_adjust_backward_refer_line_s);
DECLARE_double(planning_self_adjust_forward_refer_line_s);
DECLARE_double(planning_self_adjust_finish_distance_threshold);

//--------pose adjust function--------
DECLARE_bool(planning_pose_adjust_enable_flag);
DECLARE_int32(planning_pose_adjust_trigger_conts);
DECLARE_double(planning_pose_adjust_trigger_probability);
DECLARE_double(planning_pose_adjust_trigger_max_heading_diff);
DECLARE_double(planning_pose_adjust_front_s_range);
DECLARE_double(planning_pose_adjust_max_dis_to_bound);
DECLARE_double(planning_pose_adjust_min_dis_to_bound);
// pose adjust line function
DECLARE_double(planning_pose_adjust_backward_refer_line_s);
DECLARE_double(planning_pose_adjust_forward_refer_line_s);
DECLARE_double(planning_pose_adjust_finish_distance_threshold);
DECLARE_double(planning_pose_adjust_driving_max_distance);

//--------station stop function--------
DECLARE_double(planning_station_look_forward_dis);
DECLARE_double(planning_station_enable_detour_min_dis);
DECLARE_double(planning_station_pull_over_look_forward_dis);
DECLARE_bool(planning_station_enable_bias_driving);

//--------human interface function--------
DECLARE_int32(planning_human_interface_trigger_conts);
DECLARE_double(planning_human_interface_trigger_probability);

//--------lane borrow function--------
DECLARE_bool(planning_lane_borrow_enable_flag);
DECLARE_double(planning_lane_borrow_ratio);
DECLARE_int32(planning_lane_borrow_mode);
DECLARE_bool(planning_lane_borrow_aggressive_mode);
DECLARE_bool(planning_lane_borrow_obs_trigger_flag);
DECLARE_int32(planning_lane_borrow_obs_trigger_mode);
DECLARE_double(planning_lane_borrow_preview_s);
DECLARE_double(planning_lane_borrow_lateral_space_threshold);
DECLARE_int32(planning_lane_borrow_trigger_conts);
DECLARE_double(planning_lane_borrow_trigger_probability);
DECLARE_double(planning_lane_borrow_ROI_backward_s);
DECLARE_double(planning_lane_borrow_ROI_forward_s);
DECLARE_double(planning_lane_borrow_ROI_time);
DECLARE_double(planning_lane_borrow_reset_distance);

//--------shift function--------
DECLARE_bool(planning_shift_enable_flag);
DECLARE_double(planning_shift_normal_distance);
DECLARE_double(planning_shift_pull_over_residual_distance);
DECLARE_bool(planning_shift_broad_road_enable_flag);
DECLARE_double(planning_shift_broad_road_define_threshold);
DECLARE_double(planning_shift_broad_road_residual_distance);

//--------change lane function--------
DECLARE_bool(planning_change_lane_obs_trigger_flag);
DECLARE_double(planning_change_lane_min_distance);
DECLARE_double(planning_change_lane_front_clear_range);
DECLARE_double(planning_change_lane_back_clear_range);
DECLARE_double(planning_change_lane_dist);

//--------driving mode adjust function--------
DECLARE_int32(planning_driving_mode_min_mode);
DECLARE_int32(planning_driving_mode_max_mode);

//--------freespace function--------
DECLARE_bool(planning_enable_freespace);
DECLARE_int32(planning_freespace_go_to_line_mode);
DECLARE_int32(planning_freespace_away_line_mode);
DECLARE_int32(planning_freespace_trigger_conts);
DECLARE_double(planning_freespace_trigger_probability);
DECLARE_double(planning_freespace_backward_refer_line_s);
DECLARE_double(planning_freespace_forward_refer_line_s);
DECLARE_bool(planning_enable_freespace_refer_line);
DECLARE_int32(planning_freespace_refer_line_mode);
DECLARE_double(planning_freespace_refer_line_trigger_disance_tolerance);
DECLARE_double(planning_freespace_refer_line_trigger_heading_diff_min);
DECLARE_double(planning_freespace_refer_line_trigger_heading_diff_max);
DECLARE_bool(planning_enable_freespace_nudge);
DECLARE_double(planning_freespace_nudge_forward_dis);
DECLARE_double(planning_freespace_nudge_space_threshold);

//------------parking in function-----------
// TODO
DECLARE_bool(planning_enable_parking_in);
DECLARE_int32(planning_parking_in_trigger_conts);
DECLARE_double(planning_parking_in_trigger_probability);

//--------piecewise jerk function--------
DECLARE_double(planning_piecewise_jerk_not_nudge_distance);

//--------human near vechilce protection function--------
DECLARE_bool(planning_human_near_vehicle_enable_flag);
DECLARE_double(planning_human_near_vehicle_lateral_distance);

//--------1d lidar->lms protection function--------
DECLARE_bool(planning_enable_lms);
DECLARE_double(planning_enable_lms_speed_threshold);
DECLARE_double(planning_lms_lateral_range);
DECLARE_double(planning_lms_horizontal_range);

//--------road border protection function--------
DECLARE_double(planning_speed_border_shrink_dis);
DECLARE_double(planning_speed_border_first_segment_shrink_dis);
DECLARE_double(planning_speed_border_uturn_forward_dis);

//--------Motion planning decider--------
DECLARE_double(planning_change_planner_vel_threshold);

//--------Traffic law decider--------
DECLARE_bool(planning_enable_traffic_light_law);
DECLARE_bool(planning_enable_crosswalk_law);
DECLARE_bool(planning_enable_clear_zone_law);
DECLARE_bool(planning_enable_stop_sign_law);
DECLARE_bool(planning_enable_yield_sign_law);
DECLARE_bool(planning_enable_junction_creep_law);
// Traffic light law
DECLARE_bool(enable_traffic_light_time_count);
// Crosswalk law
DECLARE_bool(planning_construct_virtual_crosswalk);
DECLARE_int32(planning_crosswalk_obstacle_watch_frames);
DECLARE_double(planning_crosswalk_expand_distance);
DECLARE_double(planning_crosswalk_max_distance_to_ignore_pedestrian);
DECLARE_double(planning_min_distance_between_wall_and_front_edge);
DECLARE_double(planning_crosswalk_passing_line_buffer);
DECLARE_double(planning_crosswalk_look_forward_distance);
DECLARE_double(planning_crosswalk_adc_l_expand_buffer);
DECLARE_double(planning_crosswalk_watch_area_l_expand_buffer);
DECLARE_double(planning_crosswalk_roadbound_l_expand_buffer);
DECLARE_double(planning_crosswalk_crosswalk_s_expand_buffer);
DECLARE_double(planning_crosswalk_wait_time_buffer);
DECLARE_double(planning_crosswalk_adc_obs_s_speed_safe_delta);

// perception lanes
DECLARE_bool(planning_enable_perception_lanes);

// Clear zone law
DECLARE_double(planning_distance_from_check_point_to_clear_zone);
// Stop sign law
DECLARE_double(planning_stop_sign_look_forward_distance);
DECLARE_double(planning_stop_sign_stop_time_threshold);
DECLARE_bool(planning_virtual_construct_stop_and_yield_sign);
// Junction creep law
DECLARE_double(planning_junction_look_forward_distance);
DECLARE_double(planning_junction_with_stop_or_yield_sign_distance);

//--------temporary stop--------
DECLARE_double(planning_parking_stop_max_deceleration);
DECLARE_double(planning_arrive_to_destination_distance_threshold);
DECLARE_double(planning_adc_stop_velocity_threshold);
DECLARE_double(planning_destination_obstacle_distance_threshold);
DECLARE_double(planning_stgraph_stop_virtual_object_mapper_buffer);

//--------turn lights signal--------
DECLARE_double(planning_turn_lights_start_to_move_distance_threshold);
DECLARE_double(planning_turn_lights_pull_over_distance_threshold);
DECLARE_bool(planning_turn_lights_enable_nudge_obs);

//--------speed protection--------
DECLARE_bool(planning_use_chassis_speed_instead_of_localization_speed);
DECLARE_bool(planning_enable_hdmap_speed_limit);
DECLARE_double(planning_emergency_slow_down_brake_ratio);
DECLARE_bool(planning_enable_extra_speed_protection);

//--------speed planning configure--------
DECLARE_bool(planning_enable_creep_speed_process);
DECLARE_bool(planning_enable_linear_smoother);
DECLARE_bool(planning_enable_geometric_speed_smoother);
DECLARE_bool(planning_enable_speed_history_data_smoother);
DECLARE_double(planning_destination_max_forward_distance);
DECLARE_bool(planning_use_all_prediction_trajectory);
DECLARE_double(planning_prediction_trust_time_length);
DECLARE_double(planning_speed_plan_enlarge_self_buffer);
DECLARE_double(planning_speed_plan_aggressive_enlarge_self_buffer);
DECLARE_double(planning_speed_plan_risk_enlarge_self_buffer);
DECLARE_double(planning_speed_plan_enlarge_obs_buffer);
DECLARE_bool(planning_enable_geometric_speed_aggressive_mode);
DECLARE_bool(planning_enable_speed_tuning_for_control);
DECLARE_bool(planning_enable_lateral_tuning_for_control);
DECLARE_bool(planning_enable_lateral_tuning_smoother);

//--------blind area--------
DECLARE_bool(planning_enable_blind_area);
DECLARE_double(planning_blind_area_polygon_shadow_distance);
DECLARE_double(planning_blind_area_virtual_speed);
DECLARE_double(planning_blind_area_response_time);

//--------pull over function--------
DECLARE_bool(planning_enable_destination_pull_over);
DECLARE_double(planning_pull_over_require_min_distance);

//--------watch area function--------
DECLARE_double(planning_watch_area_preview_distance);
DECLARE_int64(planning_watch_area_error_threshold);
DECLARE_double(planning_nudge_decision_min_l_offset);
DECLARE_int64(planning_nudgeable_tracking_frame_num);

//--------parameters for path planning--------
DECLARE_double(planning_road_border_remain_threshold);
DECLARE_double(planning_check_path_result_max_length);
DECLARE_bool(planning_enable_check_path_result);
DECLARE_double(planning_cruise_mode_obs_expand_dis);
DECLARE_double(planning_other_mode_obs_expand_dis);
DECLARE_double(planning_nudge_prefer_lat_dis_to_obs);
DECLARE_bool(planning_piecewise_use_auto_warm_start);
DECLARE_bool(planning_open_pass_by_decider);
DECLARE_double(planning_pass_by_back_obs_velo);
DECLARE_double(planning_pass_by_side_obs_velo);
DECLARE_double(planning_pass_by_front_obs_velo);
DECLARE_uint64(planning_dp_planner);
DECLARE_uint64(planning_piecewise_planner);
DECLARE_uint64(planning_geo_planner);
DECLARE_uint64(planning_dummy_planner);
DECLARE_double(planning_piecewise_change_back_em_velo);
DECLARE_double(planning_piecewise_high_speed);
DECLARE_double(planning_piecewise_middle_high_speed);
DECLARE_double(planning_piecewise_middle_speed);
DECLARE_double(planning_piecewise_low_speed);
DECLARE_double(planning_piecewise_ulti_low_speed);
DECLARE_double(planning_piecewise_high_speed_delta_s);
DECLARE_double(planning_piecewise_low_speed_delta_s);
DECLARE_double(planning_multi_tunnels_extra_shrink_threshold);
DECLARE_uint64(planning_initial_planner);
DECLARE_uint64(planning_backup_planner);
DECLARE_uint64(planning_safe_fail_planner);
DECLARE_uint64(planning_low_consumption_lateral_sample_num);
DECLARE_bool(planning_em_path_use_qp);
DECLARE_bool(planning_only_provide_reference_line);
DECLARE_double(planning_path_planner_dynamic_obs_trust_time);

// nudge
DECLARE_double(planning_dynamic_obstacle_nudge_heading_threshold);
DECLARE_double(planning_dynamic_obstacle_nudge_extend_s);
DECLARE_double(planning_obstacle_ignore_behind_adc_extend_s);
DECLARE_double(planning_front_obstacle_ignore_speed_ratio);

//--------parameters for trajectory planning--------
DECLARE_double(planning_trajectory_min_length);
DECLARE_double(planning_trajectory_time_length);
DECLARE_double(planning_output_trajectory_time_resolution);
DECLARE_bool(planning_enable_invalid_trajectory_for_control);
DECLARE_bool(planning_enable_invalid_trajectory_for_view);

//--------parameters for trajectory sanity check--------
DECLARE_double(planning_sanity_check_epsilon);
DECLARE_double(planning_speed_lower_bound);
DECLARE_double(planning_speed_upper_bound);
DECLARE_double(planning_longitudinal_acceleration_lower_bound);
DECLARE_double(planning_longitudinal_acceleration_upper_bound);
DECLARE_double(planning_lateral_acceleration_bound);
DECLARE_double(planning_lateral_jerk_bound);
DECLARE_double(planning_longitudinal_jerk_lower_bound);
DECLARE_double(planning_longitudinal_jerk_upper_bound);
DECLARE_double(planning_kappa_bound);

//--------parameters for obs buffer--------
DECLARE_double(planning_static_decision_ignore_range);
DECLARE_double(planning_dp_path_decision_buffer);
DECLARE_double(planning_piecewise_obs_expand_buffer);
DECLARE_double(planning_piecewise_lane_expand_buffer);
DECLARE_double(planning_piecewise_circle_distance);
DECLARE_double(planning_path_adc_enlarge_buffer);
DECLARE_double(planning_dp_path_lateral_base_buffer);
DECLARE_double(planning_dp_path_lateral_control_deviation);
DECLARE_double(planning_dp_path_lateral_perception_deviation);
DECLARE_double(planning_dp_path_lateral_localization_deviation);
DECLARE_double(planning_nudge_decision_buffer);
DECLARE_double(planning_obstacle_st_boundary_buffer_default);
DECLARE_double(planning_obstacle_stop_buffer);
DECLARE_double(planning_dp_path_turn_disable_nudge_kappa_threshold);
DECLARE_double(planning_dp_path_turn_disable_nudge_distance_threshold);
DECLARE_double(planning_parking_static_decision_stop_buffer);
DECLARE_double(planning_inlane_uturn_static_decision_stop_front_buffer);

// reroute
DECLARE_string(planning_planning_reroute_topic);
DECLARE_int32(planning_change_lane_trigger_rerouting_conts);
DECLARE_double(planning_change_lane_trigger_rerouting_probability);
DECLARE_double(planning_change_lane_trigger_rerouting_dis_threshold);

// IoV
DECLARE_bool(planning_clear_tasks_on_remote_driving);
DECLARE_double(planning_clear_routing_time_threshold);

// outside data history size
DECLARE_int32(planning_speed_context_size);

// ------coordinate magager decider---------
DECLARE_bool(planning_enable_lidar_boarders_function);
DECLARE_bool(planning_enable_visual_lanes_function);

// path region and goal sl
DECLARE_bool(planning_enable_path_region_and_goal_sl);

// piecewise jerk path unequal interval sampling
DECLARE_bool(planning_enable_unequal_interval_sampling);

// low speed obstacles bypass
DECLARE_bool(planning_enable_low_speed_obstacles_by_pass);

// enable freespace scenario
DECLARE_bool(planning_is_freespace_scenario_work);

// navigation
DECLARE_bool(navigation_check_is_on_route);