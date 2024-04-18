#include "trajectory_stitcher.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {

TrajectoryStitcher::TrajectoryStitcher() {}

ErrorCode TrajectoryStitcher::stitch(
    const VehicleStateProxy& vehicle_state,
    const ControlCommand& control_command, const Frame* const prev_frame,
    const double curr_time, const double planning_cycle_time,
    std::vector<TrajectoryPoint>* const stitching_trajectory) {
  if (stitching_trajectory == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  stitching_trajectory->clear();
  is_use_position_stitch_ = false;

  /// Replan rule
  // 1. Decouple forbidden
  if (!FLAGS_use_decouple) {
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 2. Driving mode
  if (vehicle_state.DrivingMode() !=
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
    LOG_INFO("Replan: not in auto driving mode yet.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 3. Gear status
  if (vehicle_state.Gear() == neodrive::global::status::GEAR_INVALID) {
    LOG_INFO("Replan: aeb is triggered.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 4. No history frame
  if (prev_frame == nullptr) {
    LOG_INFO("Replan: prev_frame is null.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 5. Low speed
  if (vehicle_state.LinearVelocity() <
      FLAGS_planning_adc_stop_velocity_threshold) {
    LOG_INFO("Replan: low speed.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 6. Previous planning is invalid
  const auto& prev_trajectory =
      prev_frame->planning_data().computed_trajectory();
  if (prev_trajectory.num_of_points() < 3) {
    LOG_INFO("Replan: Projected trajectory at time [{}] size less than 3.",
             prev_trajectory.header_time());
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 7. Control request replan
  if (control_command.has_contrl_context() &&
      control_command.contrl_context().has_replan_request() &&
      control_command.contrl_context().replan_request()) {
    if (DataCenter::Instance()->last_frame() != nullptr &&
        DataCenter::Instance()
                ->last_frame()
                ->outside_planner_data()
                .speed_fail_tasks < 1) {
      LOG_INFO("Replan: control request replan.");
      BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
      return ErrorCode::PLANNING_OK;
    } else {
      LOG_INFO("last_frame is nullptr or trigger backup, do not replan");
    }
  }
  // 8. Compute vel_rel_time index and time_matched_point
  const double vel_rel_time = curr_time - prev_trajectory.header_time();
  std::size_t time_matched_index{0};
  if (!prev_trajectory.query_relative_time_lower_bound_index(
          vel_rel_time, time_matched_index)) {
    LOG_INFO("Replan: cal time_matched_index failed.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  if (time_matched_index == 0 &&
      vel_rel_time + kMathEpsilon <
          prev_trajectory.trajectory_points().front().relative_time()) {
    LOG_INFO("Replan: curr_time  smaller than prev_trajectory's first time.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  if (time_matched_index + 1 >= prev_trajectory.num_of_points()) {
    LOG_INFO("Replan: curr_time beyond prev_trajectory's last time.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  TrajectoryPoint time_matched_point;
  if (!prev_trajectory.trajectory_point_at(time_matched_index,
                                           time_matched_point)) {
    LOG_INFO("Replan: get time_matched_point failed.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  if (!time_matched_point.has_path_point()) {
    LOG_INFO("Replan: time_matched_point has not path_point.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 9. Compute position matched_index
  std::size_t position_matched_index{0};
  if (!prev_trajectory.query_nearest_point(
          {vehicle_state.X(), vehicle_state.Y()}, position_matched_index)) {
    LOG_INFO("Replan: compute position_matched_index failed.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 10. Compute forward_time and forward_time_index
  double forward_rel_time = vel_rel_time + planning_cycle_time;
  std::size_t forward_time_index{0};
  if (!prev_trajectory.query_relative_time_lower_bound_index(
          forward_rel_time, forward_time_index)) {
    LOG_INFO("Replan: compute forward_time_index failed.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }
  // 11. Parking start
  auto& task_info = DataCenter::Instance()->mutable_task_info_list()->front();
  if (task_info.current_frame() &&
      task_info.current_frame()->inside_planner_data().curr_scenario_state ==
          ScenarioState::PARKING &&
      task_info.last_frame() &&
      task_info.last_frame()->inside_planner_data().curr_scenario_state !=
          ScenarioState::PARKING) {
    LOG_INFO("Replan: Parking");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }

  if (task_info.last_frame() &&
      task_info.last_frame()->inside_planner_data().curr_scenario_state ==
          ScenarioState::PARKING &&
      DataCenter::Instance()->master_info().curr_stage() != "FINISH") {
    LOG_INFO("Replan: Parking");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
    return ErrorCode::PLANNING_OK;
  }

  /// Compute stitch trajectory
  LOG_INFO("Position matched index: {}", position_matched_index);
  LOG_INFO("Time matched index: {}", time_matched_index);
  LOG_INFO("Forward time matched index: {}", forward_time_index);

  auto matched_index =
      std::fmax(0, std::fmin(position_matched_index, time_matched_index));
  for (std::size_t i = matched_index;
       i <= forward_time_index && i < prev_trajectory.num_of_points(); ++i) {
    TrajectoryPoint point{};
    if (!prev_trajectory.trajectory_point_at(i, point)) {
      LOG_INFO("Replan: get {} point from prev_trajectory.", i);
      BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
      return ErrorCode::PLANNING_OK;
    }
    if (!point.has_path_point()) {
      LOG_INFO("Replan: in [{}] point has not path_point.", i);
      BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
      return ErrorCode::PLANNING_OK;
    }
    if (stitching_trajectory->empty()) {
      stitching_trajectory->emplace_back(point);
    } else if (std::abs(stitching_trajectory->back().relative_time() -
                        point.relative_time()) > 0.001) {
      stitching_trajectory->emplace_back(point);
    }
  }

  if (stitching_trajectory->empty()) {
    LOG_INFO("Replan: duplicate.");
    BuildReplanStitchingTrajectory(vehicle_state, stitching_trajectory);
  } else {
    const auto zero_s = stitching_trajectory->back().s();
    for (auto& tp : *stitching_trajectory) {
      tp.set_relative_time(tp.relative_time() + prev_trajectory.header_time() -
                           curr_time);
      tp.set_s(tp.s() - zero_s);
    }
  }

  return ErrorCode::PLANNING_OK;
}

void TrajectoryStitcher::BuildReplanStitchingTrajectory(
    const VehicleStateProxy& vehicle_state,
    std::vector<TrajectoryPoint>* const stitching_trajectory) {
  TrajectoryPoint init_point;

  init_point.set_x(vehicle_state.X());
  init_point.set_y(vehicle_state.Y());
  init_point.set_velocity(vehicle_state.LinearVelocity());
  if (vehicle_state.LinearVelocity() <
      FLAGS_planning_adc_stop_velocity_threshold + kMathEpsilon) {
    LOG_INFO(
        "vehicle velocity is {:.4f}. "
        "FLAGS_planning_adc_stop_velocity_threshold is {:.4f}. "
        "Set init acc to zero.",
        vehicle_state.LinearVelocity(),
        FLAGS_planning_adc_stop_velocity_threshold);
    init_point.set_acceleration(0.0);
  } else {
    init_point.set_acceleration(vehicle_state.LinearAcceleration());
  }
  init_point.set_theta(vehicle_state.Heading());
  init_point.set_kappa(vehicle_state.Curvature());
  init_point.set_s(0.0);
  init_point.set_relative_time(0.0);
  stitching_trajectory->emplace_back(init_point);
  is_use_position_stitch_ = true;
}

}  // namespace planning
}  // namespace neodrive
