#include "planning_gflags.h"

//--------common configure--------
DEFINE_double(planning_still_obstacle_speed_threshold, 0.1,
              "The threshodl to determine if the obstacle is still");
DEFINE_double(planning_still_pedestrian_speed_threshold, 0.1,
              "The threshodl to determine if the pedestrian is still");

//--------PlanningManager--------
DEFINE_int32(planning_max_history_result, 10,
             "The maximal number of result in history.");
DEFINE_int32(planning_period_frequence, 100, "freq of planning[Hz]");

//--------finite state machine--------
DEFINE_int32(planning_state_fail_threshold, 5,
             "This is continuous fail threshold for FSM change to fail state.");

//--------planning record-------------
DEFINE_bool(playing_record, false, "This is planning record sim flagModule");
DEFINE_bool(planning_piecewise_use_four_circles_model, false,
            "true to use four circles model, false to use pt model");
DEFINE_bool(planning_piecewise_use_soft_constraints, false,
            "true to use soft constraints");

// narrow mode
DEFINE_bool(planning_use_narrow_crossing, false,
            "true to use narrow corridor crossing mode");

//--------spend time record----------
DEFINE_bool(enable_record_spend_time, false, "enable record spend time");

//--------channels--------

DEFINE_string(planning_localization_reader_config_file,
              "/planning_localization_reader.conf",
              "reader config for localization");

DEFINE_bool(planning_enable_publish_odometry_debug_flag, true, "true--enable");

// planning test channels
DEFINE_bool(planning_test_path_publish, false, "true to open");
DEFINE_bool(planning_test_speed_publish, false, "true to open");
DEFINE_bool(planning_enable_vis_event, false, "true to open");

//--------worldwide flag--------
DEFINE_bool(planning_default_left_right_side, true,
            "vehicle running on left or right, true: right, false: left");

//--------obstacle macro define--------
DEFINE_int32(planning_object_table_obstacle_capacity, 1000,
             "The number of obstacles we hold in the object table.");
DEFINE_double(planning_polygon_length_box_length_max_diff, 2.0,
              "max length diff of polygon and box for checking sl point is "
              "right in uturn");
DEFINE_int32(planning_obstacle_record_history_frame_cnt, 3,
             "number of history frames that matter decision transit");
DEFINE_bool(planning_obstacle_enable_time_diff, true, "true to enable");
DEFINE_double(planning_obstacle_max_time_diff, 10,
              "if time diff larger than this, ignore the obs");

// virtual obstacle
DEFINE_double(planning_virtual_obstacle_length, 0.5,
              "length of destination virtual object");
DEFINE_double(planning_virtual_obstacle_max_width, 8.0,
              "max width of virtual object");
DEFINE_double(planning_virtual_obstacle_min_width, 3.5,
              "min width of virtual object");
DEFINE_double(planning_virtual_obstacle_height, 2, "height of virtual object");

//--------trajectory stitching--------
DEFINE_bool(planning_use_stitch, false, "Use trajectory stitch if possible.");
DEFINE_bool(planning_use_stitch_with_async, false,
            "Use async trajectory stitch if possible.");
DEFINE_bool(planning_enable_time_stitch, true,
            "Use time trajectory stitch if possible.");
DEFINE_double(planning_path_predict_time, 0.2,
              "The path predict time in each planning cycle.");
DEFINE_double(planning_speed_predict_time, 0.0,
              "The speed predict time in each planning cycle.");
DEFINE_double(planning_forward_predict_time, 0.2,
              "The forward predict time in each planning cycle.");
DEFINE_double(planning_last_trajectory_stub_time, 2.0,
              "Time of stub trajectory in each planning cycle.");
DEFINE_double(planning_replan_distance_threshold, 5.0,
              "The distance threshold of replan");
DEFINE_double(planning_replan_heading_threshold, M_PI / 2,
              "The heading difference to real heading threshold of replan");
DEFINE_int32(planning_stitch_tracking_buffer_size, 50,
             "stitch tracking buffer size.");
DEFINE_double(planning_stitch_moving_distance_threshold, 0.5,
              "stitch moving distance threshold.");
// control feedback trigger replan
DEFINE_bool(use_decouple, false, "use decouple.");
DEFINE_bool(planning_use_hermite_spline_interpolate, false,
            "method of trajectory interpolation");

//--------reference line--------
DEFINE_double(planning_new_routing_start_s_max_diff, 100,
              "adc current position and new routing start point max diff");
DEFINE_double(planning_reference_line_extend_length, 12.0,
              "extend reference line distance");
DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");
DEFINE_double(invalid_reference_line_heading_diff_threshold, 1.23,
              "invalid reference line heading diff threshold");
DEFINE_double(routing_point_gap, 1.0, "distance between points");
DEFINE_double(reference_line_start_s, -10.0,
              "start pos of ref line to curr pos");
DEFINE_double(reference_line_end_s, 50.0, "end pos of ref line to curr pos");
DEFINE_double(reference_line_dis_threshold, 200,
              "max distance threshold for projection");
// refer line in decision
DEFINE_double(planning_look_forward_short_distance, 10.0,
              "plane forward distance");
DEFINE_double(planning_look_forward_long_distance, 20.0,
              "plane forward in high speed");
DEFINE_double(planning_look_backward_distance, 10.0,
              "backward distance to ref");
DEFINE_bool(reference_line_enable_precise_road_bound_calculation, false,
            "true to enable");

//--------reverse driving--------
DEFINE_bool(planning_enable_reverse_driving, false, "enable reverse driving.");
DEFINE_int32(
    planning_reverse_driving_trigger_conts, 50,
    "when vehicle unexpected parking during cruise wait for 40 *0.1 s, "
    "try to reverse");
DEFINE_double(planning_reverse_driving_trigger_probability, 0.8,
              "probability to enable reverse");
DEFINE_double(planning_reverse_driving_front_l_range, 1.5,
              "front lateral range, val-0.7");
DEFINE_double(planning_reverse_driving_front_s_range, 7.0,
              "front longitudinal range val - 2.0");
DEFINE_double(planning_reverse_driving_back_lat_thresh, 1.8,
              "back longitudinal range");
DEFINE_bool(planning_enable_reverse_tuining_control, false,
            "true to force enable reverse");
// reverse line function
DEFINE_double(planning_reverse_driving_max_distance, 6.0,
              "max reverse distance.");
DEFINE_double(planning_reverse_driving_finish_distance_threshold, 0.5,
              "If the distance between vehicle and destination point smaller "
              "than this threshold, reverse driving finished.");

//--------inlane uturn--------
DEFINE_bool(planning_enable_inlane_uturn, false,
            "enable in lane u turn function");
DEFINE_bool(planning_inlane_uturn_ignore_obs, false, "true ignore obs to plan");
DEFINE_double(planning_inlane_uturn_adjust_length, 10.0,
              "adjust min length for inlane uturn");
DEFINE_double(planning_inlane_uturn_front_buffer, 2.0,
              "buffer for trajectory generation");
DEFINE_double(planning_inlane_uturn_rear_buffer, 2.0,
              "buffer for trajectory generation");
DEFINE_double(planning_inlane_uturn_left_buffer, 2.0,
              "buffer for trajectory generation");
DEFINE_double(planning_inlane_uturn_right_buffer, 2.0,
              "buffer for trajectory generation");
DEFINE_int32(planning_inlane_uturn_trigger_conts, 20,
             "wait for 20 *0.1 s to enter uturn");
DEFINE_double(planning_inlane_uturn_trigger_probability, 0.6,
              "probability to enable inlane uturn");

// inlane uturn refer line
DEFINE_double(planning_inlane_uturn_min_width, 4.5,
              "min width to do in lane uturn");
DEFINE_double(planning_inlane_uturn_backward_refer_line_s, 20.0,
              "backward distance");
DEFINE_double(planning_inlane_uturn_forward_refer_line_s, 20.0,
              "forward distance");
DEFINE_double(planning_inlane_uturn_reach_distance_threshold, 0.2,
              "reach distance");
DEFINE_double(planning_inlane_uturn_reach_heading_threshold, 0.1,
              "reach heading");
DEFINE_double(planning_inlane_uturn_min_radius, 5.0, "min radius");
DEFINE_double(planning_inlane_uturn_shrink_border_dis, 0.3, "shrink distance");
DEFINE_bool(planning_enable_inlane_uturn_obs_trigger, false,
            "enable in lane u turn triggered by obs");
DEFINE_double(planning_inlane_uturn_front_clear_s, 5.0, "front-s clear zone");
DEFINE_double(planning_inlane_uturn_rear_clear_s, 5.0, "rear-s clear zone");
DEFINE_double(planning_inlane_uturn_left_clear_l, 4.0, "left-l clear zone");
DEFINE_double(planning_inlane_uturn_right_clear_l, 4.0, "right-l clear zone");

//--------out lane u-turn--------
DEFINE_bool(planning_enable_outlane_uturn, false,
            "enable outlane uturn function");
DEFINE_double(planning_outlane_uturn_trigger_radius, 3.6,
              "if radius less than this, trigger outlane uturn");
DEFINE_int32(planning_outlane_uturn_trigger_conts, 20,
             "wait for 20 *0.1 s to enter uturn");
DEFINE_double(planning_outlane_uturn_trigger_probability, 0.6,
              "probability to enable outlane uturn");
DEFINE_double(planning_outlane_uturn_backward_refer_line_s, 20.0,
              "backward distance");
DEFINE_double(planning_outlane_uturn_forward_refer_line_s, 20.0,
              "forward distance");
DEFINE_double(planning_outlane_uturn_reach_distance_threshold, 0.2,
              "reach distance");
DEFINE_double(planning_outlane_uturn_reach_heading_threshold, 0.1,
              "reach heading");

//--------self adjust function--------
DEFINE_bool(planning_self_adjust_enable_flag, false,
            "enable self_adjust function");
DEFINE_double(planning_self_adjust_trigger_min_heading_diff, 0.34,
              "min heading diff to trigger self_adjust");
DEFINE_double(planning_self_adjust_trigger_max_heading_diff, 1.47,
              "min heading diff to trigger self_adjust");
DEFINE_bool(planning_self_adjust_enable_cross_road_bound_flag, false,
            "enable self_adjust cross road_bound function");
DEFINE_double(planning_self_adjust_cross_road_bound_buffer, 0.0,
              "max dis cross road bound");
DEFINE_double(planning_self_adjust_forward_clear_zone_s, 5.0,
              "forward enlarge s");
DEFINE_double(planning_self_adjust_backward_clear_zone_s, 5.0,
              "backward enlarge s");
DEFINE_double(planning_self_adjust_lateral_clear_zone_l, 1.0,
              "lateral enlarge l");
DEFINE_int32(
    planning_self_adjust_trigger_conts, 40,
    "when vehicle unexpected parking during cruise wait for 40 *0.1 s, "
    "try to self adjust");
DEFINE_double(planning_self_adjust_trigger_probability, 0.8,
              "probability to enable self adjust");
// self adjust line function
DEFINE_double(planning_self_adjust_backward_refer_line_s, 10.0,
              "backward distance");
DEFINE_double(planning_self_adjust_forward_refer_line_s, 10.0,
              "forward distance");
DEFINE_double(planning_self_adjust_finish_distance_threshold, 1.0,
              "If the distance between vehicle and destination point smaller "
              "than this threshold, self_adjust driving finished.");

//--------pose adjust function--------
DEFINE_bool(planning_pose_adjust_enable_flag, false,
            "enable pose adjust function");
DEFINE_int32(
    planning_pose_adjust_trigger_conts, 60,
    "when vehicle unexpected parking due to road board protection wait for "
    "40 *0.1 s, try to pose adjust");
DEFINE_double(planning_pose_adjust_trigger_probability, 0.8,
              "probability to enable pose adjust");
DEFINE_double(planning_pose_adjust_trigger_max_heading_diff, 1.47,
              "min heading diff to trigger pose adjust");
DEFINE_double(planning_pose_adjust_front_s_range, 7.0,
              "front longitudinal range val");
DEFINE_double(planning_pose_adjust_max_dis_to_bound, 1.5,
              "max distance of adc boundary to road bound");
DEFINE_double(planning_pose_adjust_min_dis_to_bound, -0.1,
              "min distance of adc boundary to road bound");
// pose adjust line function
DEFINE_double(planning_pose_adjust_backward_refer_line_s, 10.0,
              "backward distance");
DEFINE_double(planning_pose_adjust_forward_refer_line_s, 10.0,
              "forward distance");
DEFINE_double(planning_pose_adjust_finish_distance_threshold, 1.0,
              "If the distance between vehicle and destination point smaller "
              "than this threshold, self_adjust driving finished.");
DEFINE_double(planning_pose_adjust_driving_max_distance, 6.0,
              "max reverse distance.");

//--------station stop function--------
DEFINE_double(planning_station_look_forward_dis, 25.0,
              "min look forward distance");
DEFINE_double(planning_station_enable_detour_min_dis, 10.0,
              "min distance which enable detour near stop station.");
DEFINE_double(planning_station_pull_over_look_forward_dis, 30.0,
              "min look forward distance");
DEFINE_bool(planning_station_enable_bias_driving, true,
            "in station mode, enable bias driving");

//--------human interface function--------
DEFINE_int32(planning_human_interface_trigger_conts, 15,
             "counter to smooth state of human interface");
DEFINE_double(planning_human_interface_trigger_probability, 0.6,
              "probability to trigger");

//--------lane borrow function--------
DEFINE_bool(planning_lane_borrow_enable_flag, false, "lane borrow to nudge");
DEFINE_double(planning_lane_borrow_ratio, 0.8, "ration of next lane to use");
DEFINE_int32(planning_lane_borrow_mode, 0,
             "0: no-borrow; 1: white-dashed; 2: white-solid; 3: yellow-dashed, "
             "4: yellow-solid; 5: double yellow-solid");
DEFINE_bool(planning_lane_borrow_aggressive_mode, false,
            "true: enable aggressive mode, can borrow space instead of lane");
DEFINE_bool(planning_lane_borrow_obs_trigger_flag, false,
            "borrow solid lane(white/yellow) due to static obs");
DEFINE_int32(planning_lane_borrow_obs_trigger_mode, 4,
             "0: no-borrow; 1: white-dashed; 2: white-solid; 3: yellow-dashed, "
             "4: yellow-solid; 5: double yellow-solid");
DEFINE_double(planning_lane_borrow_preview_s, 25.0, "preview dis");
DEFINE_double(
    planning_lane_borrow_lateral_space_threshold, 1.6,
    "lateral space, if obs to lane bound less than this, lane borrow");
DEFINE_int32(planning_lane_borrow_trigger_conts, 20, "wait for 20 * 0.1s");
DEFINE_double(planning_lane_borrow_trigger_probability, 0.8,
              "probability to trigger borrow solid lane");
DEFINE_double(planning_lane_borrow_ROI_backward_s, 10.0, "distance");
DEFINE_double(planning_lane_borrow_ROI_forward_s, 10.0, "ditance");
DEFINE_double(planning_lane_borrow_ROI_time, 3.0, "time");
DEFINE_double(planning_lane_borrow_reset_distance, 100.0, "distance");

//--------shift function--------
DEFINE_bool(planning_shift_enable_flag, false, "true to enable");
DEFINE_double(planning_shift_normal_distance, -0.5,
              "normal shift value, left +, right -");
DEFINE_double(planning_shift_pull_over_residual_distance, 0.25,
              "residual distance to boarder");
DEFINE_bool(planning_shift_broad_road_enable_flag, true, "true to enable");
DEFINE_double(planning_shift_broad_road_define_threshold, 8.0, "threshold");
DEFINE_double(planning_shift_broad_road_residual_distance, 2.4,
              "residual sitance to boarder");

//--------change lane function--------
DEFINE_bool(planning_change_lane_obs_trigger_flag, false, "true to enable");
DEFINE_double(
    planning_change_lane_min_distance, 10,
    "target lane left distance must larger than this when changing lane");
DEFINE_double(planning_change_lane_front_clear_range, 10.0,
              "change lane front clear range");
DEFINE_double(planning_change_lane_back_clear_range, 3.0,
              "change lane back clear range");
DEFINE_double(planning_change_lane_dist, 38,
              "start to change lane before change lane end point.");

//--------driving mode adjust function--------
DEFINE_int32(planning_driving_mode_min_mode, 1, "min mode");
DEFINE_int32(planning_driving_mode_max_mode, 3, "max mode");

//--------freespace function--------
DEFINE_bool(planning_enable_freespace, false, "enable freespace function");
DEFINE_int32(planning_freespace_go_to_line_mode, 0,
             "mode: 0: target inside of init freespace; 1: target can outside "
             "of init freespace");
DEFINE_int32(planning_freespace_away_line_mode, 0,
             "mode: 0: target inside of init freespace; 1: target can outside "
             "of init freespace");
DEFINE_int32(planning_freespace_trigger_conts, 40,
             "wait for 40 *0.1 s, "
             "try to trigger freespace");
DEFINE_double(planning_freespace_trigger_probability, 0.8,
              "probability to enable freespace");
DEFINE_double(planning_freespace_backward_refer_line_s, 20.0,
              "backward distance");
DEFINE_double(planning_freespace_forward_refer_line_s, 20.0,
              "forward distance");
DEFINE_bool(planning_enable_freespace_refer_line, false,
            "enable to go to the line or away from line");
DEFINE_double(planning_freespace_refer_line_trigger_disance_tolerance, 20.0,
              "if distance larger than this, donot trigger");
DEFINE_double(planning_freespace_refer_line_trigger_heading_diff_min, -0.52,
              "-30 deg");
DEFINE_double(planning_freespace_refer_line_trigger_heading_diff_max, 2.09,
              "120 deg");

DEFINE_int32(planning_freespace_refer_line_mode, 0,
             "reference line trigger mode, 0: to line, 1: away from line, 2: "
             "to/away line");
DEFINE_bool(planning_enable_freespace_nudge, false,
            "enable use freespace to nudge");
DEFINE_double(planning_freespace_nudge_forward_dis, 15.0,
              "forward distance to trigger nudge");
DEFINE_double(planning_freespace_nudge_space_threshold, 2.3,
              "if space larger than this, donot trigger");

//-------------parking in function------------
DEFINE_bool(planning_enable_parking_in, false, "enable parking in function");
DEFINE_int32(planning_parking_in_trigger_conts, 40,
             "wait for 40 * 0.1s, "
             "try to trigger parking in");  // mchan doubt
DEFINE_double(planning_parking_in_trigger_probability, 0.8,
              "probability to enable parking in");

//--------piecewise jerk function--------
DEFINE_double(planning_piecewise_jerk_not_nudge_distance, 3.0,
              "vehicle front 2m dis do not avoid.");

//--------human near vechilce protection function--------
DEFINE_bool(planning_human_near_vehicle_enable_flag, false, "true to enable");
DEFINE_double(planning_human_near_vehicle_lateral_distance, 0.3,
              "distance to detect human ");

//--------1d lidar->lms protection function--------
DEFINE_bool(planning_enable_lms, false, "enable lms");
DEFINE_double(planning_enable_lms_speed_threshold, 5.0,
              "if adc speed more than this value, don not use lms data [m/s]");
DEFINE_double(planning_lms_lateral_range, 3.5,
              "lms left/right lateral valid range.");
DEFINE_double(planning_lms_horizontal_range, 3.5, "lms horizontal valid range");

//--------road border protection function--------
DEFINE_double(planning_speed_border_shrink_dis, 0.2,
              "shrink border dis only in speed planning");
DEFINE_double(planning_speed_border_first_segment_shrink_dis, -0.1,
              "shrink border dis only in speed planning");
DEFINE_double(planning_speed_border_uturn_forward_dis, 0.65,
              "valid uturn/parking forward distance");

//--------Motion planning decider--------
DEFINE_double(planning_change_planner_vel_threshold, 1.389,
              "if vehicle speed lower than this, change planner");

//--------Traffic law decider--------
DEFINE_bool(planning_enable_traffic_light_law, true,
            "enable traffic light law.");
DEFINE_bool(planning_enable_crosswalk_law, false, "enable crosswalk law.");
DEFINE_bool(planning_enable_clear_zone_law, false, "enable clear zone law.");
DEFINE_bool(planning_enable_stop_sign_law, false, "enable stop sign law.");
DEFINE_bool(planning_enable_yield_sign_law, false, "enable yield sign law.");
DEFINE_bool(planning_enable_junction_creep_law, false,
            "enable junction creep law.");
// Traffic light law
DEFINE_bool(enable_traffic_light_time_count, false,
            "enable time in traffic light.");
// Crosswalk law
DEFINE_bool(planning_construct_virtual_crosswalk, false,
            "construct virtual crosswalk for test");
DEFINE_int32(planning_crosswalk_obstacle_watch_frames, 120,
             "watch frames for obstacle on crosswalk");
DEFINE_double(planning_crosswalk_expand_distance, 2.0,
              "crosswalk expand distance(planning_meter) for "
              "pedestrian/bicycle detection");
DEFINE_double(
    planning_crosswalk_max_distance_to_ignore_pedestrian, 6.5,
    "max l_distance to ignore pedestrian on crosswalk when path not crosses");
DEFINE_double(planning_min_distance_between_wall_and_front_edge, 0.1,
              "min distance between wall and front edge");
DEFINE_double(planning_crosswalk_passing_line_buffer, 1.5,
              "passing stop line buffer length");
DEFINE_double(planning_crosswalk_look_forward_distance, 30.0,
              "crosswalk look forward distance threshold in meter.");
DEFINE_double(planning_crosswalk_adc_l_expand_buffer, 0.2,
              "expand buffer of adc");
DEFINE_double(planning_crosswalk_watch_area_l_expand_buffer, 4.5,
              "expand buffer of watch area");
DEFINE_double(planning_crosswalk_roadbound_l_expand_buffer, 0.5,
              "expand buffer of road bound");
DEFINE_double(planning_crosswalk_crosswalk_s_expand_buffer, 1.0,
              "expand buffer of crosswalk");
DEFINE_double(planning_crosswalk_wait_time_buffer, 5.0,
              "largest wait time for certain crosswalk");
DEFINE_double(planning_crosswalk_adc_obs_s_speed_safe_delta, 0.2,
              "ignore obs if its speed on s direction faster than adc.");

// perception lanes
DEFINE_bool(planning_enable_perception_lanes, false, "true for open");

// Clear zone law
DEFINE_double(planning_clear_zone_look_forward_distance, 30.0,
              "clearzone look forward distance threshold in meter.");
DEFINE_double(planning_distance_from_check_point_to_clear_zone, 5,
              "check point distance (planning_meters) after keep clear zone");
// Stop sign law
DEFINE_double(planning_stop_sign_look_forward_distance, 30.0,
              "stop sign look forward distance threshold in meter");
DEFINE_double(planning_stop_sign_stop_time_threshold, 3.0,
              "stop sign stop time threshold in second");
DEFINE_bool(planning_virtual_construct_stop_and_yield_sign, false,
            "construct virtual stop and yield sign for test");
// Junction creep law
DEFINE_double(planning_junction_look_forward_distance, 30.0,
              "junction look forward distance threshold in meter");
DEFINE_double(planning_junction_with_stop_or_yield_sign_distance, 30.0,
              "junction with yield sign distance threshold in meter.");

//--------temporary stop--------
DEFINE_double(planning_parking_stop_max_deceleration, 1.0,
              "parking stop max deceleration");
DEFINE_double(
    planning_arrive_to_destination_distance_threshold, 1.0,
    "if the adc to destination is less than this value, auto driving finish.");
DEFINE_double(
    planning_adc_stop_velocity_threshold, 0.2,
    "if the adc to destination is less than this value, auto driving finish");
DEFINE_double(planning_destination_obstacle_distance_threshold, 0.5,
              "if the adc to obstacle of destination is less than this value, "
              "auto driving finish.");
DEFINE_double(planning_stgraph_stop_virtual_object_mapper_buffer, 0.3,
              "The default s up/down buffer for virtual object mapping");

//--------turn lights signal--------
DEFINE_double(planning_turn_lights_start_to_move_distance_threshold, 2.0,
              "if start to move and move distance less than this value, open "
              "left signal");
DEFINE_double(planning_turn_lights_pull_over_distance_threshold, 15,
              "if distance to destination less this value and need pull over, "
              "open right signal.");
DEFINE_bool(planning_turn_lights_enable_nudge_obs, false, "true to enable");

//--------speed protection--------
DEFINE_bool(planning_use_chassis_speed_instead_of_localization_speed, false,
            "true: use chassis speed, false: use localization speed");
DEFINE_bool(planning_enable_hdmap_speed_limit, true,
            "enable speed limit from hdmap");
DEFINE_double(planning_emergency_slow_down_brake_ratio, 0.5,
              "emergency brake ratio when plan failed.");
DEFINE_bool(planning_enable_extra_speed_protection, true, "true to enable");

//--------speed planning configure--------
DEFINE_bool(planning_enable_creep_speed_process, true, "enable it");
DEFINE_bool(planning_enable_linear_smoother, false,
            "use linear smoother to smooth dp st speed");
DEFINE_bool(planning_enable_geometric_speed_smoother, false,
            "use smoother to smooth geo speed");
DEFINE_bool(planning_enable_speed_history_data_smoother, false,
            "enable history data's smoother");
DEFINE_double(planning_destination_max_forward_distance, 50,
              "set destination max forward distance");
DEFINE_bool(planning_use_all_prediction_trajectory, false,
            "true to use all trajecotrys, false only use first");
DEFINE_double(planning_prediction_trust_time_length, 3.0,
              "time to use prediction in speed planning");
DEFINE_double(planning_speed_plan_enlarge_self_buffer, 0.4,
              "enlarge self buffer");
DEFINE_double(planning_speed_plan_aggressive_enlarge_self_buffer, 0.3,
              "aggressive mode, enlarge self buffer");
DEFINE_double(planning_speed_plan_risk_enlarge_self_buffer, 0.2,
              "risk mode, enlarge self buffer");
DEFINE_double(planning_speed_plan_enlarge_obs_buffer, 0.1,
              "enlarge obs buffer");
DEFINE_bool(planning_enable_geometric_speed_aggressive_mode, true,
            "true: enable");
DEFINE_bool(planning_enable_speed_tuning_for_control, false,
            "true to enable speed tuning for control");
DEFINE_bool(planning_enable_lateral_tuning_for_control, false,
            "true to enable lateral tuning for control");
DEFINE_bool(planning_enable_lateral_tuning_smoother, true, "enable smoother");

//--------blind area--------
DEFINE_bool(planning_enable_blind_area, false, "enable blind area strategy.");
DEFINE_double(planning_blind_area_polygon_shadow_distance, 5.0,
              "blind area polygon shadow distance.");
DEFINE_double(planning_blind_area_virtual_speed, 2.78,
              "blind area virtual speed.");
DEFINE_double(planning_blind_area_response_time, 0.5,
              "blind area response time.");

//--------pull over function--------
DEFINE_bool(planning_enable_destination_pull_over, false,
            "enable destination pull over");
DEFINE_double(planning_pull_over_require_min_distance, 20.0,
              "pull over require min distance");

//--------watch area function--------
DEFINE_double(planning_watch_area_preview_distance, 10.0,
              "watch area preview distance [m]");
DEFINE_int64(planning_watch_area_error_threshold, 10,
             "watch area error frame number threshold");
DEFINE_double(planning_nudge_decision_min_l_offset, 0.5,
              "nudge decision min l offset [m]");
DEFINE_int64(planning_nudgeable_tracking_frame_num, 10,
             "nudgeable frame tracking number");

//--------parameters for path planning--------
DEFINE_double(planning_road_border_remain_threshold, 0.3,
              "the min distance between adc and road border.");
DEFINE_double(planning_check_path_result_max_length, 0.6,
              "check path invalid max length.");
DEFINE_bool(planning_enable_check_path_result, false,
            "enable check path result.");
DEFINE_double(planning_cruise_mode_obs_expand_dis, 0.3,
              "path plan obs expand dis in cruise mode.");
DEFINE_double(planning_other_mode_obs_expand_dis, 0.2,
              "path plan obs expand dis in other mode.");
DEFINE_double(planning_nudge_prefer_lat_dis_to_obs, 0.4,
              "path plan obs expand dis in other mode.");
DEFINE_uint64(planning_dp_planner, 0,
              "0: em, 1: piecewise jerk, 2: geo, 99: dummy");
DEFINE_uint64(planning_piecewise_planner, 1,
              "0: em, 1: piecewise jerk, 2: geo, 99: dummy");
DEFINE_uint64(planning_geo_planner, 2,
              "0: em, 1: piecewise jerk, 2: geo, 99: dummy");
DEFINE_uint64(planning_dummy_planner, 99,
              "0: em, 1: piecewise jerk, 2: geo, 99: dummy");
DEFINE_double(planning_piecewise_change_back_em_velo, 1.0,
              "if velo > 1.0m/s, piecewise planner failed, "
              "can change back to first planner or change into third planner");
DEFINE_double(planning_piecewise_high_speed, 2.5, "m/s");
DEFINE_double(planning_piecewise_middle_high_speed, 2.0, "m/s");
DEFINE_double(planning_piecewise_middle_speed, 1.5, "m/s");
DEFINE_double(planning_piecewise_low_speed, 1.0, "m/s");
DEFINE_double(planning_piecewise_ulti_low_speed, 0.7, "m/s");
DEFINE_double(planning_piecewise_high_speed_delta_s, 1.0, "m");
DEFINE_double(planning_piecewise_low_speed_delta_s, 0.6, "m");
DEFINE_double(
    planning_multi_tunnels_extra_shrink_threshold, 2.0,
    "mutitunnels extra shrink distance threshold, if < 2.0m, donot shrink");
DEFINE_uint64(planning_initial_planner, 0,
              "0: initial, 1: backup, 2: safe fail");
DEFINE_uint64(planning_backup_planner, 1,
              "0: initial, 1: backup, 2: safe fail");
DEFINE_uint64(planning_safe_fail_planner, 2,
              "0: initial, 1: backup, 2: safe fail");
DEFINE_uint64(planning_low_consumption_lateral_sample_num, 5,
              "low consumpltion lateral sample num");
DEFINE_bool(
    planning_em_path_use_qp, false,
    "for em path, true to use qp path, false to not use qp, just use dp.");
DEFINE_bool(planning_only_provide_reference_line, false,
            "for control testing, just give reference line.");
DEFINE_double(planning_path_planner_dynamic_obs_trust_time, 0.1,
              "trust time for dynamic obs in path planner");

// planning piecewise
DEFINE_bool(planning_piecewise_use_auto_warm_start, true,
            "for piecewise path qp, true to use warm start between frame, "
            "false not use");

// pass by scenario
DEFINE_bool(planning_open_pass_by_decider, false,
            "true: open pass by scenario");
DEFINE_double(planning_pass_by_back_obs_velo, 2.0,
              "m/s, back obs overtake adc, velo threshold");
DEFINE_double(planning_pass_by_side_obs_velo, 2.0,
              "m/s, side obs overtake adc, velo threshold");
DEFINE_double(planning_pass_by_front_obs_velo, 2.0,
              "m/s, front reverse obs meet adc, velo threshold");

DEFINE_double(
    planning_dynamic_obstacle_nudge_heading_threshold, M_PI * 0.167,
    "if the diff heading between adc and dynamic obstacle more than this value,"
    "do not nudge this obstacle");
DEFINE_double(planning_dynamic_obstacle_nudge_extend_s, 6.0,
              "extend dynamic obstacle predict trajectory for nudge.");
DEFINE_double(planning_obstacle_ignore_behind_adc_extend_s, 10.0,
              "obstacle ignore behind adc extend_s.");
DEFINE_double(planning_front_obstacle_ignore_speed_ratio, 0.6,
              "obstacle ignore front speed ratio.");

//--------parameters for trajectory planning--------
DEFINE_double(planning_trajectory_min_length, 30, "Trajectory min length");
DEFINE_double(planning_trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(planning_output_trajectory_time_resolution, 0.02,
              "Trajectory time resolution when publish");
DEFINE_bool(planning_enable_invalid_trajectory_for_control, true,
            "true: send invalid trajectory for control");
DEFINE_bool(planning_enable_invalid_trajectory_for_view, false,
            "true: send invalid trajectory for view");

//--------parameters for trajectory sanity check--------
DEFINE_double(planning_sanity_check_epsilon, 1e-4,
              "The checker range bound epsilon value");
DEFINE_double(planning_speed_lower_bound, 0.0, "The lowest speed allowed.");
DEFINE_double(planning_speed_upper_bound, 40.0, "The highest speed allowed.");
DEFINE_double(planning_longitudinal_acceleration_lower_bound, -5.0,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(planning_longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(
    planning_lateral_acceleration_bound, 4.5,
    "The bound of lateral acceleration; symmetric for left and right");
DEFINE_double(planning_lateral_jerk_bound, 4.0,
              "The bound of lateral jerk; symmetric for left and right");
DEFINE_double(planning_longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(planning_longitudinal_jerk_upper_bound, 4.0,
              "The upper bound of longitudinal jerk.");
DEFINE_double(planning_kappa_bound, 0.23, "The bound for vehicle curvature");

//--------parameters for obs buffer--------
DEFINE_double(planning_static_decision_ignore_range, 2.0,
              "threshold for judging nudge in dp path computing decision");
DEFINE_double(planning_dp_path_decision_buffer, 0.3,
              "buffer in decision while dp path computing decision");
DEFINE_double(planning_piecewise_obs_expand_buffer, 0.241,
              "buffer for piecewise obs lateral expand");
DEFINE_double(planning_piecewise_lane_expand_buffer, 0.241,
              "buffer for piecewise lane lateral expand");
DEFINE_double(planning_piecewise_circle_distance, 0.25,
              "first circle center distance to back edge");
DEFINE_double(planning_path_adc_enlarge_buffer, 0.2,
              "path planner adc bounding box enlarge buffer");
DEFINE_double(planning_dp_path_lateral_base_buffer, 0.15,
              "base lateral buffer for decision");
DEFINE_double(planning_dp_path_lateral_control_deviation, 0.1,
              "lateral control deviation for decision");
DEFINE_double(planning_dp_path_lateral_perception_deviation, 0.1,
              "lateral perception deviation for decision");
DEFINE_double(planning_dp_path_lateral_localization_deviation, 0.1,
              "lateral localization deviation for decision");
DEFINE_double(planning_nudge_decision_buffer, 0.40,
              "buffer in decision while do nudge decision");
DEFINE_double(planning_obstacle_st_boundary_buffer_default, 1.0,
              "buffer when yield or follow an obstacle");
DEFINE_double(planning_obstacle_stop_buffer, 2.0,
              "minimal buffer when stop after an obstacle");
DEFINE_double(planning_dp_path_turn_disable_nudge_kappa_threshold, 0.12,
              "disable nudge for curve if max kappa exceeded this value.");
DEFINE_double(
    planning_dp_path_turn_disable_nudge_distance_threshold, 5.0,
    "disable nudge for curve with specific distance and max kappa exceeded.");
DEFINE_double(planning_parking_static_decision_stop_buffer, 0.2,
              "parking static decision stop buffer");

DEFINE_double(planning_inlane_uturn_static_decision_stop_front_buffer, 0.5,
              "front stop decision buffer for inlane uturn action.");

//  reroute
DEFINE_string(planning_planning_reroute_topic, "/router/routing_signal", "");
DEFINE_int32(planning_change_lane_trigger_rerouting_conts, 30,
             "histroy data counts");
DEFINE_double(planning_change_lane_trigger_rerouting_probability, 0.8,
              "probability to trigger rerouting");
DEFINE_double(planning_change_lane_trigger_rerouting_dis_threshold, 5.0,
              "distance threshold");

// IoV
DEFINE_bool(planning_clear_tasks_on_remote_driving, true,
            "true-clear task; false-do nothing");
DEFINE_double(planning_clear_routing_time_threshold, 5.0, "time threshold");

// outside data history size
DEFINE_int32(planning_speed_context_size, 10,
             "planning OutSidePlannerData-speed-context size.");

// ------coordinate magager decider---------
DEFINE_bool(planning_enable_lidar_boarders_function, false, "true--enable");
DEFINE_bool(planning_enable_visual_lanes_function, false, "true--enable");

// path region and goal sl
DEFINE_bool(planning_enable_path_region_and_goal_sl, false, "true-enable");

// piecewise jerk path unequal interval sampling
DEFINE_bool(planning_enable_unequal_interval_sampling, false, "true-enable");

// low speed obstacles bypass
DEFINE_bool(planning_enable_low_speed_obstacles_by_pass, false, "true-enable");

// enable freespace scenario
DEFINE_bool(planning_is_freespace_scenario_work, false,
            "whether freespace scenario work");

// navigation
DEFINE_bool(navigation_check_is_on_route, true,
            "whether to check ego car is on route lane");