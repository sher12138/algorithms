#include "indoor_speed_filter_decider.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kSqrRadiusLidar = 6.4 * 6.4;
constexpr double kLongRange = 6.0;
}  // namespace

IndoorSpeedFilterDecider::IndoorSpeedFilterDecider() {
  name_ = "IndoorSpeedFilterDecider";
}

void IndoorSpeedFilterDecider::Reset() {}

IndoorSpeedFilterDecider::~IndoorSpeedFilterDecider() { Reset(); }

void IndoorSpeedFilterDecider::SaveTaskResults(TaskInfo& task_info) {}

ErrorCode IndoorSpeedFilterDecider::Execute(TaskInfo& task_info) {
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  LOG_INFO(">>>> start execute {}", name_);
  const auto& indoor_config =
      config::PlanningConfig::Instance()->plan_config().indoor;
  const auto ego_polygon = VehicleParam::Instance()->get_adc_polygon(
      {inside_data.vel_x, inside_data.vel_y}, inside_data.vel_heading, 0.0, 0.0,
      0.0);
  auto lidar_pts = GetLidarPoints(*(data_center_->lidar_freespace_msg.ptr));
  if (lidar_pts.empty()) {
    LOG_ERROR("Lidar freespace is empty, skip");
    return ErrorCode::PLANNING_OK;
  }
  Polygon2d freespace_polygon(lidar_pts);
  const auto& obs_vec =
      task_info.current_frame()->planning_data().decision_data().all_obstacle();
  auto& dp_st_map_ignore_static_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_static_obs_id;
  auto& dp_st_map_ignore_dynamic_obs_id =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  for (const auto obs_ptr : obs_vec) {
    if (obs_ptr->is_virtual()) continue;
    if (obs_ptr->speed() > indoor_config.freespace_filter_speed_threshold)
      continue;
    if (obs_ptr->polygon().distance_to(ego_polygon) >
        indoor_config.lidar_freespace_max_distance) {
      continue;
    }
    if (obs_ptr->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      continue;
    }
    double length{0.};
    for (const auto& seg : obs_ptr->polygon().segments()) {
      length += SegmentLengthInsidePolygon(freespace_polygon, seg);
    }
    if (length > indoor_config.freespace_filter_length_threshold) {
      if (obs_ptr->is_static()) {
        dp_st_map_ignore_static_obs_id.insert(obs_ptr->id());
        LOG_INFO(
            "Ignore static obs [{}] because of indoor freespace filter, "
            "length = {:.2f}",
            obs_ptr->id(), length);
      } else {
        dp_st_map_ignore_dynamic_obs_id.insert(obs_ptr->id());
        LOG_INFO(
            "Ignore dynamic obs [{}] because of indoor freespace filter, "
            "length = {:.2f}",
            obs_ptr->id(), length);
      }
    }
  }
  return ErrorCode::PLANNING_OK;
}

std::vector<Vec2d> IndoorSpeedFilterDecider::GetLidarPoints(
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

double IndoorSpeedFilterDecider::SegmentLengthInsidePolygon(
    const Polygon2d& polygon, const Segment2d& segment) const {
  std::vector<Vec2d> intersections;
  const auto& points = polygon.points();
  for (size_t i = 0; i < points.size(); ++i) {
    Segment2d edge{points[i], points[(i + 1) % points.size()]};
    Vec2d intersect;
    if (edge.get_intersect(segment, &intersect)) {
      intersections.emplace_back(intersect);
    }
  }
  intersections.emplace_back(segment.start());
  intersections.emplace_back(segment.end());
  std::sort(intersections.begin(), intersections.end(),
            [&](const Vec2d& a, const Vec2d& b) {
              return a.distance_to(segment.start()) <
                     b.distance_to(segment.start());
            });
  double length{0.0};
  for (size_t i = 0; i < intersections.size() - 1; ++i) {
    Vec2d mid{(intersections[i].x() + intersections[i + 1].x()) / 2.,
              (intersections[i].y() + intersections[i + 1].y()) / 2.};
    if (polygon.is_point_in(mid)) {
      length += intersections[i].distance_to(intersections[i + 1]);
    }
  }
  return length;
}

}  // namespace planning
}  // namespace neodrive
