#include "indoor_speed_avoid_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

IndoorSpeedAvoidDecider::IndoorSpeedAvoidDecider() {
  name_ = "IndoorSpeedAvoidDecider";
}

void IndoorSpeedAvoidDecider::Reset() {}

IndoorSpeedAvoidDecider::~IndoorSpeedAvoidDecider() { Reset(); }

void IndoorSpeedAvoidDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode IndoorSpeedAvoidDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  const auto& obs_vec =
      task_info.current_frame()->planning_data().decision_data().all_obstacle();
  bool avoid{false};
  double preview_dis{5.};
  std::vector<Polygon2d> ego_polygon_vec;
  auto* outside_data =
      task_info.current_frame()->mutable_outside_planner_data();
  if (outside_data->path_data == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  const auto& path_points = outside_data->path_data->path().path_points();
  for (const auto& path_point : path_points) {
    if (path_point.s() > preview_dis) {
      break;
    }
    auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        {path_point.x(), path_point.y()}, path_point.theta(), 0.0, 0.0, 0.0);
    ego_polygon_vec.emplace_back(ego_polygon);
  }
  for (const auto* obs_ptr : obs_vec) {
    if (obs_ptr == nullptr || obs_ptr->is_virtual() || obs_ptr->is_static()) {
      continue;
    }
    if (obs_ptr->center_sl().l() > 0.) {
      continue;
    }
    const auto& pred_traj = obs_ptr->uniform_trajectory();
    for (std::size_t j = 0; j < pred_traj.trajectory_points().size(); ++j) {
      TrajectoryPoint point{};
      if (!pred_traj.trajectory_point_at(j, point)) {
        LOG_ERROR("trajectory point at {} failed", j);
        continue;
      }
      Polygon2d obs_polygon = Utility::get_trajectory_point_polygon(
          obs_ptr->center(), {point.x(), point.y()},
          obs_ptr->velocity_heading(), point.theta(), obs_ptr->polygon());

      if (std::any_of(ego_polygon_vec.begin(), ego_polygon_vec.end(),
                      [&obs_polygon](const Polygon2d& ego_polygon) {
                        return obs_polygon.has_overlap(ego_polygon);
                      })) {
        avoid = true;
        LOG_INFO("indoor avoid right obs {}", obs_ptr->id());
        break;
      }
    }
    if (avoid) break;
  }
  if (avoid) {
    neodrive::global::planning::SpeedLimit new_limit{};
    new_limit.set_source_type(SpeedLimitType::REF_LINE);
    new_limit.add_upper_bounds(0.);
    new_limit.set_constraint_type(SpeedLimitType::SOFT);
    new_limit.set_acceleration(0.);
    LOG_DEBUG("{} {} limit speed: speed = {:.2f}, acc = {:.2f}",
              SpeedLimit_SourceType_Name(new_limit.source_type()),
              SpeedLimit_ConstraintType_Name(new_limit.constraint_type()),
              new_limit.upper_bounds().at(0) * 3.6, new_limit.acceleration());
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(new_limit);
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
