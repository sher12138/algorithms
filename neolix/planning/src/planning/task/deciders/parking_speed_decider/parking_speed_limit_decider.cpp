#include "parking_speed_limit_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

ParkingSpeedLimitDecider::ParkingSpeedLimitDecider() {
  name_ = "ParkingSpeedLimitDecider";
}

void ParkingSpeedLimitDecider::Reset() {}

ParkingSpeedLimitDecider::~ParkingSpeedLimitDecider() { Reset(); }

void ParkingSpeedLimitDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedLimitDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  const auto& inside_data = frame->inside_planner_data();
  const auto& parking_config =
      config::PlanningConfig::Instance()->plan_config().parking;
  double parking_max_speed{parking_config.parking_in_max_speed};

  double parking_speed_limit{std::min(
      frame->outside_planner_data().parking_outside_data.parking_speed_cmd,
      parking_max_speed)};

  if (data_center_->master_info().curr_stage() == "PARKING_IN") {
    if (data_center_->master_info().drive_direction() ==
        MasterInfo::DriveDirection::DRIVE_BACKWARD) {
      parking_speed_limit =
          std::fmin(parking_speed_limit, parking_config.backward_max_speed);
      if (data_center_->parking_ptr()->OriginPark()->Type() ==
          global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
        parking_speed_limit = std::fmin(
            parking_speed_limit,
            parking_config.horizontal_parking_space.backward_max_speed);
      }

    } else {
      parking_speed_limit =
          std::fmin(parking_speed_limit, parking_config.forward_max_speed);
      if (data_center_->parking_ptr()->OriginPark()->Type() ==
          global::hdmap::ParkingSpace_ParkingType_HORIZONTAL) {
        parking_speed_limit = std::fmin(
            parking_speed_limit,
            parking_config.horizontal_parking_space.forward_max_speed);
      }
    }
  } else if (data_center_->master_info().curr_stage() == "PARKING_OUT") {
    ;
  }
  if (data_center_->master_info().curr_stage() == "FINISH") {
    parking_speed_limit = parking_config.forward_max_speed;
  }
  parking_speed_limit = std::min(
      frame->outside_planner_data().parking_outside_data.parking_speed_cmd,
      parking_max_speed);

  neodrive::global::planning::SpeedLimit new_limit{};
  new_limit.set_source_type(SpeedLimitType::REF_LINE);
  // frame->outside_planner_data().parking_outside_data.parking_speed_cmd;
  new_limit.add_upper_bounds(parking_speed_limit);
  new_limit.set_constraint_type(SpeedLimitType::HARD);
  new_limit.set_acceleration(0.);
  LOG_INFO("parking speed limit to {:.2f}",
           new_limit.upper_bounds().at(0) * 3.6);
  data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(new_limit);

  if (std::min(
          frame->outside_planner_data().parking_outside_data.parking_speed_cmd,
          parking_max_speed) < 0.06) {
    frame->mutable_outside_planner_data()->parking_stop = true;
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
