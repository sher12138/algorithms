#include "parking_speed_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

ParkingSpeedDecider::ParkingSpeedDecider() { name_ = "ParkingSpeedDecider"; }

void ParkingSpeedDecider::Reset() {}

ParkingSpeedDecider::~ParkingSpeedDecider() { Reset(); }

void ParkingSpeedDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode ParkingSpeedDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  const auto& indoor_config =
      config::PlanningConfig::Instance()->plan_config().indoor;
  // if (indoor_config.enable_obstacle) {
  //   return ErrorCode::PLANNING_OK;
  // }

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
  if (!lidar_pts.empty()) {
    for (std::size_t i = 0; i < path_points.size(); ++i) {
      auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
          path_points[i], path_points[i].theta(), 0.0, 0.0, 0.0);
      if (!freespace_polygon.is_contain(ego_polygon) &&
          freespace_polygon.has_overlap(ego_polygon)) {
        freespace_max_s = path_points[i].s();
        break;
      }
    }
  }
  LOG_INFO("freespace_max_s = {:.2f}", freespace_max_s);
  std::size_t size_post_data{81};
  // if (path_points.empty() ||
  //     task_info.current_frame()->outside_planner_data().parking_stop) {
  //   upper_s_bounds.clear();
  //   lower_s_bounds.clear();
  //   upper_v_bounds.clear();
  //   lower_v_bounds.clear();
  //   goal_decision_data.Reset();
  //   for (std::size_t i = 0; i < size_post_data; ++i) {
  //     upper_s_bounds.emplace_back(
  //         STBoundInfo(STPoint(freespace_max_s - 0.5, i * 0.1),
  //                     STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
  //     lower_s_bounds.emplace_back(
  //         STBoundInfo(STPoint(0.0, i * 0.1),
  //                     STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
  //     STGoalVInfo u_v_bound{
  //         .goal_t_v = STPoint(1.39, i * 0.1),
  //         .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
  //     upper_v_bounds.emplace_back(u_v_bound);

  //     STGoalVInfo l_v_bound{
  //         .goal_t_v = STPoint(0.0, i * 0.1),
  //         .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN};
  //     lower_v_bounds.emplace_back(l_v_bound);
  //     goal_decision_data.deduction_ego_a_sequence.emplace_back(-2.);
  //     goal_decision_data.deduction_ego_v_sequence.emplace_back(
  //         std::max(inside_data.init_point.velocity() - i * 0.2, 0.0));
  //     goal_decision_data.deduction_ego_p_sequence.emplace_back(0.);
  //     goal_decision_data.deduction_ego_t_sequence.emplace_back(i * 0.1);
  //   }
  // }
  // if (data_center_->parking_ptr() &&
  //     data_center_->parking_ptr()->is_park_in() &&
  //     data_center_->parking_ptr()->OriginPark() &&
  //     data_center_->parking_ptr()->OriginPark()->Type() ==
  //         global::hdmap::ParkingSpace_ParkingType_VERTICAL) {
  //   const auto& speed_limits =
  //       data_center_->behavior_speed_limits().speed_limits();
  //   const auto& speed_limits_enable =
  //       data_center_->behavior_speed_limits().speed_limits_enable();
  //   double v_upper_bound{1.};
  //   for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
  //     if (!speed_limits_enable[i]) {
  //       continue;
  //     }
  //     const auto& speed_limit = speed_limits[i];
  //     if (speed_limit.constraint_type() == SpeedLimitType::HARD &&
  //         speed_limit.upper_bounds_size() == 1) {
  //       LOG_INFO("speed limit is [{}]", speed_limit.upper_bounds().at(0));
  //       v_upper_bound =
  //           std::min(v_upper_bound, speed_limit.upper_bounds().at(0));
  //     }
  //   }
  //   upper_v_bounds.clear();
  //   for (std::size_t i = 0; i < size_post_data; ++i) {
  //     upper_v_bounds.emplace_back(
  //         STGoalVInfo{.goal_t_v = STPoint(v_upper_bound, i * 0.1),
  //                     .boundary_type =
  //                     STGraphBoundary::BoundaryType::UNKNOWN});
  //   }
  // }

  const auto& speed_limits =
      data_center_->behavior_speed_limits().speed_limits();
  const auto& speed_limits_enable =
      data_center_->behavior_speed_limits().speed_limits_enable();
  double v_upper_bound{1.};
  for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
    if (!speed_limits_enable[i]) {
      continue;
    }
    const auto& speed_limit = speed_limits[i];
    if (speed_limit.constraint_type() == SpeedLimitType::HARD &&
        speed_limit.upper_bounds_size() == 1) {
      LOG_INFO("speed limit is [{}]", speed_limit.upper_bounds().at(0));
      v_upper_bound = std::min(v_upper_bound, speed_limit.upper_bounds().at(0));
    }
  }
  std::vector<SpeedPoint> speed_vector;
  double last_s{0.};
  for (std::size_t i = 0; i < 80; ++i) {
    double tmp_s{last_s + inside_data.init_point.velocity() * 0.1};
    speed_vector.emplace_back(tmp_s, i * 0.1, v_upper_bound, 0., 0.);
    last_s = tmp_s;
  }
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  outside_data->speed_data->set_speed_vector(speed_vector);
  outside_data->speed_data->mutable_st_graph_data()->set_guided_speed_data(
      speed_vector);
  outside_data->speed_context.dp_st_data.smoothed_speed = speed_vector;
  outside_data->speed_succeed_tasks += 1;
  return ErrorCode::PLANNING_OK;
}

std::vector<Vec2d> ParkingSpeedDecider::GetLidarPoints(
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
