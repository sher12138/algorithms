#include "parking_speed_reverse_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

ParkingSpeedReverseDecider::ParkingSpeedReverseDecider() {
  name_ = "ParkingSpeedReverseDecider";
}

void ParkingSpeedReverseDecider::Reset() {}

ParkingSpeedReverseDecider::~ParkingSpeedReverseDecider() { Reset(); }

void ParkingSpeedReverseDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedReverseDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  const auto& indoor_config =
      config::PlanningConfig::Instance()->plan_config().indoor;
  if (indoor_config.enable_obstacle) {
    return ErrorCode::PLANNING_OK;
  }

  if (data_center_->master_info().drive_direction() !=
      MasterInfo::DriveDirection::DRIVE_BACKWARD) {
    LOG_INFO("Ego is not driving backward, skip");
    return ErrorCode::PLANNING_OK;
  }

  auto& dp_st_data = task_info.current_frame()
                         ->mutable_outside_planner_data()
                         ->speed_context.dp_st_data;
  auto& upper_s_bounds = dp_st_data.upper_iter_bound;
  auto& lower_s_bounds = dp_st_data.lower_iter_bound;
  auto& upper_v_bounds = dp_st_data.upper_iter_v_bound;
  auto& lower_v_bounds = dp_st_data.lower_iter_v_bound;
  auto& goal_decision_data = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->speed_obstacle_context.iter_deduction;
  const auto& path_points = task_info.current_frame()
                                ->outside_planner_data()
                                .path_data->path()
                                .path_points();
  const auto& ego_boundaries = task_info.current_frame()
                                   ->outside_planner_data()
                                   .speed_obstacle_context.adc_sl_boundaries;

  const auto lidar_pts =
      GetLidarPoints(*(data_center_->lidar_freespace_msg.ptr));
  const Polygon2d freespace_polygon(lidar_pts);
  double freespace_max_s = path_points.empty() ? 0. : path_points.back().s();
  for (std::size_t i = 0; i < path_points.size(); ++i) {
    auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
        path_points[i], path_points[i].theta(), 0.0, 0.0, 0.0);
    if (freespace_polygon.has_overlap(ego_polygon)) {
      freespace_max_s = path_points[i].s();
      break;
    }
  }
  LOG_INFO("freespace_max_s = {:.2f}", freespace_max_s);
  std::size_t size_post_data{81};
  if (freespace_max_s < 3.2) {
    upper_s_bounds.clear();
    lower_s_bounds.clear();
    upper_v_bounds.clear();
    lower_v_bounds.clear();
    goal_decision_data.Reset();
    for (std::size_t i = 0; i < size_post_data; ++i) {
      upper_s_bounds.emplace_back(
          STBoundInfo(STPoint(freespace_max_s - 0.5, i * 0.1),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      lower_s_bounds.emplace_back(
          STBoundInfo(STPoint(0.0, i * 0.1),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      STGoalVInfo u_v_bound{
          .goal_t_v = STPoint(1.39, i * 0.1),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      upper_v_bounds.emplace_back(u_v_bound);

      STGoalVInfo l_v_bound{
          .goal_t_v = STPoint(0.0, i * 0.1),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      lower_v_bounds.emplace_back(l_v_bound);
      goal_decision_data.deduction_ego_a_sequence.emplace_back(-2.);
      goal_decision_data.deduction_ego_v_sequence.emplace_back(
          std::max(inside_data.init_point.velocity() - i * 0.2, 0.0));
      goal_decision_data.deduction_ego_p_sequence.emplace_back(0.);
      goal_decision_data.deduction_ego_t_sequence.emplace_back(i * 0.1);
    }
  } else if (freespace_max_s < 6.4) {
    upper_s_bounds.clear();
    lower_s_bounds.clear();
    upper_v_bounds.clear();
    lower_v_bounds.clear();
    goal_decision_data.Reset();
    for (std::size_t i = 0; i < size_post_data; ++i) {
      upper_s_bounds.emplace_back(
          STBoundInfo(STPoint(freespace_max_s - 0.5, i * 0.1),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      lower_s_bounds.emplace_back(
          STBoundInfo(STPoint(0.0, i * 0.1),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      STGoalVInfo u_v_bound{
          .goal_t_v = STPoint(1.39, i * 0.1),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      upper_v_bounds.emplace_back(u_v_bound);

      STGoalVInfo l_v_bound{
          .goal_t_v = STPoint(0.0, i * 0.1),
          .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
      lower_v_bounds.emplace_back(l_v_bound);
      goal_decision_data.deduction_ego_a_sequence.emplace_back(-2.);
      goal_decision_data.deduction_ego_v_sequence.emplace_back(
          std::max(inside_data.init_point.velocity() - i * 0.1, 0.5));
      goal_decision_data.deduction_ego_p_sequence.emplace_back(0.);
      goal_decision_data.deduction_ego_t_sequence.emplace_back(i * 0.1);
    }
  }
  return ErrorCode::PLANNING_OK;
}

std::vector<Vec2d> ParkingSpeedReverseDecider::GetLidarPoints(
    const neodrive::global::perception::Freespace& freespace) const {
  std::vector<Vec2d> lidar_points, ans;
  for (const auto& p : freespace.freespace()) {
    lidar_points.emplace_back(p.x(), p.y());
  }
  auto& pose =
      data_center_->environment().perception_proxy().Perception().odom_pose();
  auto& loc = pose.position();
  auto& q = pose.orientation();
  const double theta =
      std::atan2(2. * (q.qw() * q.qz() + q.qx() * q.qy()),
                 1. - 2. * (q.qy() * q.qy() + q.qz() * q.qz()));
  const double st{std::sin(theta)}, ct{std::cos(theta)};
  for (const auto& point : lidar_points) {
    double x_odom{point.x() * ct - point.y() * st + loc.x()};
    double y_odom{point.x() * st + point.y() * ct + loc.y()};
    ans.emplace_back(x_odom, y_odom);
  }
  return ans;
}

}  // namespace planning
}  // namespace neodrive
