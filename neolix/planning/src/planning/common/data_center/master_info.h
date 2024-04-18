#pragma once

#include "common/macros.h"
#include "common/util/time_logger.h"
#include "decision.pb.h"
#include "global_adc_status.pb.h"
#include "planning.pb.h"
#include "planning/common/data_center/decision_context.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/proxy/global_state_proxy.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class MasterInfo {
  using double_pair = std::pair<double, double>;

 public:
  MasterInfo() = default;
  ~MasterInfo() = default;

  enum class LaneChangeType {
    NONE = 0,
    LEFT_FORWARD = 1,
    RIGHT_FORWARD = 2,
    LEFT_BACKWARD = 3,
    RIGHT_BACKWARD = 4
  };

  enum class DriveDirection { DRIVE_FORWARD = 0, DRIVE_BACKWARD = 1 };

  // reset context every frame.
  void FrameReset();

  /* need to reorganie blow after change lane online*/
  bool IsChangeLaneSignalOpen() const;
  void OpenChangeLaneSignal(MasterInfo::LaneChangeType lane_change_type);
  void CloseChangeLaneSignal();
  ADCSignals::SignalType ChangeLaneTurnType() const;
  ADCSignals::SignalType LaneTurnLightType() const;

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_last_route);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_near_current_rl_end);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, behavior_stop_vehicle);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, need_bias_driving);
  // DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, bias_driving_val);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, keep_bias_distance);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, pull_over_distance_to_goal);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_station_stop);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_station_stop_need_pull_over);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_destination_in_unlaoding_zone);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_in_inner_station_mode);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_trigger_stop);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, distance_to_end);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, yield_to_end_s);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_use_position_stitch);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(DriveDirection, drive_direction);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, lane_borrow_type);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(ReferenceLinePtr, reference_line_road);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, stop_due_to_obs);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, stop_due_to_collision);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, stop_due_to_invalid_pos);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, human_take_over);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, restricted_area);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, need_yield);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(LaneChangeType, lane_change_type);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(int, change_lane_signal_open_time);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferencePoint, vehicle_point);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferencePoint, destination_point);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferencePoint,
                                       origin_destination_point);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReferencePoint,
                                       reference_line_destination_point);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(double_pair, passable_l_range);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ReverseLaneDetourContext,
                                       reverse_lane_detour_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(LaneBorrowContext, lane_borrow_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(MotorwayLaneBorrowContext,
                                       motorway_lane_borrow_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(CruiseContext, cruise_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(IntersectionContext,
                                       intersection_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(BarrierGateContext,
                                       barrier_gate_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(MotorwayCruiseContext,
                                       motorway_cruise_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(MotorwayLaneChangeContext,
                                       motorway_lane_change_context);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(MotorwayIntersectionContext,
                                       motorway_intersection_context);

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(ScenarioState::State, curr_scenario);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::string, curr_stage);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(AutoPilotIntention, curr_intention);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, is_ask_for_takeover);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, stop_due_to_junction_left_turn);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, stop_due_to_y_junction);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, clean_adc_signal);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(ADCSignals::SignalType, adc_signal);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, adc_target_speed);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, adc_target_accel);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, enable_static_detour);

 private:
  // behavior flag
  bool is_last_route_{false};
  bool is_near_current_rl_end_{false};
  bool behavior_stop_vehicle_{false};

  bool need_bias_driving_{false};
  // double bias_driving_val_{0.};
  double keep_bias_distance_{0.0};
  double pull_over_distance_to_goal_{0.0};
  bool is_station_stop_{false};
  bool is_station_stop_need_pull_over_{false};
  bool is_destination_in_unlaoding_zone_{false};
  bool is_in_inner_station_mode_{false};
  bool is_trigger_stop_{false};
  double distance_to_end_{0.};
  double yield_to_end_s_{0.};
  bool is_use_position_stitch_{false};

  ReferencePoint vehicle_point_{};
  ReferencePoint destination_point_{};
  ReferencePoint origin_destination_point_{};
  ReferencePoint reference_line_destination_point_{};

  DriveDirection drive_direction_{DriveDirection::DRIVE_FORWARD};

  std::pair<double, double> passable_l_range_{-10.0, 10.0};

  // in self adjust, uturn, freespace, use this
  // refer line to record real road
  ReferenceLinePtr reference_line_road_{nullptr};
  int lane_borrow_type_{0};  // 0:none, 1:left, 2:right

  // stop reason
  bool stop_due_to_obs_{false};
  bool stop_due_to_collision_{false};
  bool stop_due_to_invalid_pos_{false};
  bool human_take_over_{false};
  bool is_ask_for_takeover_{false};
  bool stop_due_to_junction_left_turn_{false};
  bool stop_due_to_y_junction_{false};
  bool restricted_area_{false};
  bool need_yield_{false};

  /* need to reorganie blow after change lane online*/
  LaneChangeType lane_change_type_{LaneChangeType::NONE};

  int change_lane_signal_open_time_{-1};  // unit: us

  ReverseLaneDetourContext reverse_lane_detour_context_{};
  LaneBorrowContext lane_borrow_context_{};
  MotorwayLaneBorrowContext motorway_lane_borrow_context_{};
  CruiseContext cruise_context_{};
  IntersectionContext intersection_context_{};
  BarrierGateContext barrier_gate_context_{};
  MotorwayCruiseContext motorway_cruise_context_{};
  MotorwayLaneChangeContext motorway_lane_change_context_{};
  MotorwayIntersectionContext motorway_intersection_context_{};

  ScenarioState::State curr_scenario_{ScenarioState::INIT};
  std::string curr_stage_{""};
  AutoPilotIntention curr_intention_{AutoPilotIntention::FINISH};
  bool clean_adc_signal_{true};
  ADCSignals::SignalType adc_signal_{};
  double adc_target_speed_{0.};
  double adc_target_accel_{0.};

  bool enable_static_detour_{true};
};

}  // namespace planning
}  // namespace neodrive
