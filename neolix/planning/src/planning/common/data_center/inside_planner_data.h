#pragma once

#include "src/planning/common/data_center/decision_context.h"
#include "src/planning/common/path/path_data.h"
#include "src/planning/common/speed/speed_data.h"
#include "src/planning/common/trajectory/publishable_trajectory.h"
#include "src/planning/common/trajectory/trajectory_point.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {
/*
 * To separate motion planning (task & optimizer) with decision/behavior module,
 * use the InsidePlannerData as data converter;
 */
enum class CoordinateType { UTM = 0, ODOMETRY = 1 };

struct InsidePlannerData {
  // behavior input
  CoordinateType trajectory_mode{CoordinateType::UTM};  // 0:utm; 1: odometry;
  bool is_bias_driving{false};
  double distance_to_end{0.};
  bool is_changing_lane{false};
  bool is_parking_in_slot{false};
  bool is_parking_out_slot{false};
  bool is_inlane_uturn{false};
  bool is_outlane_uturn{false};
  bool is_reverse_driving{false};
  bool is_pose_adjust{false};
  bool is_lane_borrowing{false};
  bool is_prepare_borrowing{false};
  bool is_indoor{false};
  bool is_in_the_park{false};
  LaneBorrowContext::BorrowSide lane_borrow_side{
      LaneBorrowContext::BorrowSide::None};

  std::size_t curr_multi_level{0};
  std::size_t curr_pass_by_mode{0};              // given by pass by decider
  std::size_t pre_decision_pass_by_scenario{0};  // given by pre decision
  std::size_t acceleration_mode{0};              // [0,1,2],

  // vechile enviroment data
  double vel_x{0.};
  double vel_y{0.};
  double vel_heading{0.};
  double vel_v{0.};
  double vel_a{0.};
  double vel_pitch_angle{0.};
  double vel_steer_angle{0.};  // percent

  // if trajectory_mode[ODOMETRY], use this to record utm_pose
  ReferencePoint utm_pose;
  double control_lat_error{0.};
  double control_head_error{0.};

  TrajectoryPoint init_point{};
  SLPoint init_sl_point{};

  ReferencePoint reference_line_destination_point{};

  PathData last_path_data{};
  SpeedData last_speed_data{};
  // last frame's trajectory
  PublishableTrajectory last_computed_trajectory{};

  // lms safety check
  std::vector<Vec2d> lms_pts{};

  // output
  bool speed_planner_try_again{false};  // task manager
  double valid_speed_result{0.};
  // used in change lane task mode
  // safety length of speed planner
  bool change_lane_task_mode{false};

  /// For freespace scenario
  using AD2 = std::array<double, 2>;
  neodrive::global::planning::ScenarioState::State curr_scenario_state{
      neodrive::global::planning::ScenarioState::CRUISE};
  bool is_replan{false};
  bool left_pass_obs{true};
  double back_out_end_s{0.0};
  SLPoint target_sl_point{};
  TrajectoryPoint target_point{};
  std::vector<AD2> polygon{};
  std::vector<std::vector<AD2>> holes{};
};

}  // namespace planning
}  // namespace neodrive