#include "indoor_speed_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

IndoorSpeedDecider::IndoorSpeedDecider() { name_ = "IndoorSpeedDecider"; }

void IndoorSpeedDecider::Reset() {}

IndoorSpeedDecider::~IndoorSpeedDecider() { Reset(); }

void IndoorSpeedDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode IndoorSpeedDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  const auto& indoor_config =
      config::PlanningConfig::Instance()->plan_config().indoor;
  if (indoor_config.enable_obstacle) {
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

  const auto lidar_pts =
      GetLidarPoints(*(data_center_->lidar_freespace_msg.ptr));
  double freespace_max_s = path_points.empty() ? 0. : path_points.back().s();
  if (lidar_pts.size() < 3) {
    LOG_INFO("lidar points size < 3");
  } else {
    const Polygon2d freespace_polygon(lidar_pts);
    for (const auto& point : path_points) {
      auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
          point, point.theta(), 0.0, 0.0, 0.0);
      if (freespace_polygon.is_contain(ego_polygon)) {
        continue;
      }
      if (freespace_polygon.has_overlap(ego_polygon)) {
        freespace_max_s = point.s();
        break;
      }
    }
    LOG_INFO("freespace_max_s = {:.2f}", freespace_max_s);
  }

  auto outside_ptr = task_info.current_frame()->mutable_outside_planner_data();
  std::vector<SpeedPoint> speed_vector;

  const auto& speed_limits =
      data_center_->behavior_speed_limits().speed_limits();
  const auto& speed_limits_enable =
      data_center_->behavior_speed_limits().speed_limits_enable();
  double v_limit{indoor_config.max_speed};
  for (std::size_t i = 0; i < speed_limits_enable.size(); ++i) {
    if (!speed_limits_enable[i]) {
      continue;
    }
    const auto& speed_limit = speed_limits[i];
    if (speed_limit.upper_bounds_size() == 1) {
      LOG_INFO("speed limit is [{}]", speed_limit.upper_bounds().at(0));
      v_limit = std::min(v_limit, speed_limit.upper_bounds().at(0));
    }
  }
  double v_target{v_limit};
  int speed_size = static_cast<int>(time_length_ / resolution_);
  if (freespace_max_s < indoor_config.freespace_stop_dis_threshold &&
      !path_points.empty() && freespace_max_s < path_points.back().s()) {
    outside_ptr->speed_slow_down = true;
    for (std::size_t i = 0; i < speed_size; ++i) {
      speed_vector.emplace_back(0., i * resolution_, 0., 0., 0.);
    }
    outside_ptr->speed_data->set_speed_vector(speed_vector);
    outside_ptr->speed_data->mutable_st_graph_data()->set_guided_speed_data(
        speed_vector);
    outside_ptr->speed_context.dp_st_data.smoothed_speed = speed_vector;
    outside_ptr->speed_succeed_tasks += 1;
  } else if (freespace_max_s <
             indoor_config.freespace_low_speed_dis_threshold) {
    v_target = std::fmin(v_target, indoor_config.low_speed);
    double last_s{0.};
    double last_v{inside_data.vel_v};
    for (std::size_t i = 0; i < speed_size; ++i) {
      double tmp_v{std::fmax(
          v_target,
          inside_data.vel_v + indoor_config.min_acc * i * resolution_)};
      double tmp_s{last_s + (last_v + tmp_v) / 2. * resolution_};
      speed_vector.emplace_back(tmp_s, i * resolution_, tmp_v, 0., 0.);
      last_s = tmp_s;
      last_v = tmp_v;
    }
    outside_ptr->speed_data->set_speed_vector(speed_vector);
    outside_ptr->speed_data->mutable_st_graph_data()->set_guided_speed_data(
        speed_vector);
    outside_ptr->speed_context.dp_st_data.smoothed_speed = speed_vector;
    outside_ptr->speed_succeed_tasks += 1;
  } else {
    v_target = std::fmin(v_target, indoor_config.max_speed);
    double init_v{std::max(inside_data.init_point.velocity(), 0.)};
    double last_s{0.};
    double last_v{init_v};
    double t_need = {std::abs(v_target - init_v) / .5};
    double a_need = v_target - init_v > 0.1 ? 0.5 : 0.;

    goal_decision_data.Reset();
    for (std::size_t i = 0; i < size_post_data_; ++i) {
      upper_s_bounds.emplace_back(
          STBoundInfo(STPoint(80.3, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      lower_s_bounds.emplace_back(
          STBoundInfo(STPoint(-.1, i * time_step_),
                      STGraphBoundary::BoundaryType::UNKNOWN, 0, 0.0));
      upper_v_bounds.emplace_back(
          STGoalVInfo{.goal_t_v = STPoint(v_target, i * time_step_),
                      .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN});
      lower_v_bounds.emplace_back(
          STGoalVInfo{.goal_t_v = STPoint(.0, i * time_step_),
                      .boundary_type = STGraphBoundary::BoundaryType::UNKNOWN});
      double tmp_t{i * time_step_};
      double tmp_v{std::min(init_v + a_need * tmp_t, v_target)};
      tmp_v = std::max(0., tmp_v);
      double tmp_s{last_s + (last_v + tmp_v) / 2. * time_step_};
      last_s = tmp_s;
      last_v = tmp_v;
      goal_decision_data.deduction_ego_a_sequence.emplace_back(
          tmp_t > t_need ? 0 : a_need);
      goal_decision_data.deduction_ego_p_sequence.emplace_back(tmp_s);
      goal_decision_data.deduction_ego_t_sequence.emplace_back(tmp_t);
      goal_decision_data.deduction_ego_v_sequence.emplace_back(tmp_v);
    }
  }
  return ErrorCode::PLANNING_OK;
}

std::vector<Vec2d> IndoorSpeedDecider::GetLidarPoints(
    const neodrive::global::perception::Freespace& freespace) const {
  if (freespace.freespace().empty()) return {};
  std::vector<Vec2d> lidar_points;
  auto& pose =
      data_center_->environment().perception_proxy().Perception().odom_pose();
  auto& loc = pose.position();
  auto& q = pose.orientation();
  const double theta =
      std::atan2(2. * (q.qw() * q.qz() + q.qx() * q.qy()),
                 1. - 2. * (q.qy() * q.qy() + q.qz() * q.qz()));
  const double st{std::sin(theta)}, ct{std::cos(theta)};
  for (const auto& point : freespace.freespace()) {
    double x_odom{point.x() * ct - point.y() * st + loc.x()};
    double y_odom{point.x() * st + point.y() * ct + loc.y()};
    lidar_points.emplace_back(x_odom, y_odom);
  }
  return lidar_points;
}

}  // namespace planning
}  // namespace neodrive
