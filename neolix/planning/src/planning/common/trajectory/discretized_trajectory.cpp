#include "discretized_trajectory.h"

#include "common/math/util.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"

namespace neodrive {
namespace planning {

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points)
    : trajectory_points_(trajectory_points) {}

double DiscretizedTrajectory::time_length() const {
  if (trajectory_points_.empty()) return 0.0;

  return trajectory_points_.back().relative_time() -
         trajectory_points_.front().relative_time();
}

double DiscretizedTrajectory::spatial_length() const {
  if (trajectory_points_.empty()) return 0.0;

  return trajectory_points_.back().s() - trajectory_points_.front().s();
}

bool DiscretizedTrajectory::evaluate(const double relative_time,
                                     TrajectoryPoint& pt) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }

  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);

  if (it_lower == trajectory_points_.begin()) {
    pt = trajectory_points_.front();
  } else if (it_lower == trajectory_points_.end()) {
    pt = trajectory_points_.back();
  } else {
    pt =
        TrajectoryPoint::interpolate(*(it_lower - 1), *it_lower, relative_time);
  }
  return true;
}

bool DiscretizedTrajectory::evaluate_linear_approximation(
    const double relative_time, TrajectoryPoint& pt) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);

  if (it_lower == trajectory_points_.begin()) {
    pt = trajectory_points_.front();
  } else if (it_lower == trajectory_points_.end()) {
    pt = trajectory_points_.back();
  } else {
    pt = TrajectoryPoint::interpolate_linear_approximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
  return true;
}

bool DiscretizedTrajectory::query_relative_time_lower_bound_index(
    const double relative_time, std::size_t& index) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }

  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time() < relative_time;
  };
  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, func);

  if (it_lower == trajectory_points_.end()) {
    index = std::size_t(
        (trajectory_points_.end() - trajectory_points_.begin() - 1));
  }
  index = std::size_t((it_lower - trajectory_points_.begin()));
  return true;
}

bool DiscretizedTrajectory::query_nearest_point(const Vec2d& position,
                                                std::size_t& index) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }
  double dist_min = std::numeric_limits<double>::max();
  std::size_t index_min = 0;
  for (std::size_t i = 0; i < trajectory_points_.size(); ++i) {
    Vec2d dist_vec = trajectory_points_[i].coordinate() - position;
    double dist = dist_vec.length_sqr();
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }
  index = index_min;
  return true;
}

void DiscretizedTrajectory::append_trajectory_point(
    const TrajectoryPoint& trajectory_point) {
  if (!trajectory_points_.empty()) {
    if (trajectory_point.relative_time() <=
        trajectory_points_.back().relative_time())
      return;
  }
  trajectory_points_.push_back(trajectory_point);
}

bool DiscretizedTrajectory::trajectory_point_at(const std::size_t index,
                                                TrajectoryPoint& pt) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }
  if (index >= num_of_points()) {
    pt = trajectory_points_.back();
  } else {
    pt = trajectory_points_[index];
  }
  return true;
}

bool DiscretizedTrajectory::start_point(TrajectoryPoint& pt) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }

  pt = trajectory_points_.front();
  return true;
}

bool DiscretizedTrajectory::end_point(TrajectoryPoint& pt) const {
  if (trajectory_points_.empty()) {
    LOG_ERROR("trajectory_points_.empty()");
    return false;
  }

  pt = trajectory_points_.back();
  return true;
}

std::size_t DiscretizedTrajectory::num_of_points() const {
  return trajectory_points_.size();
}

const std::vector<TrajectoryPoint>& DiscretizedTrajectory::trajectory_points()
    const {
  return trajectory_points_;
}

std::vector<TrajectoryPoint>*
DiscretizedTrajectory::mutable_trajectory_points() {
  return &trajectory_points_;
}

void DiscretizedTrajectory::set_trajectory_points(
    const std::vector<TrajectoryPoint>& points) {
  trajectory_points_ = points;
}

void DiscretizedTrajectory::insert_into_front_without_last_point(
    const std::vector<TrajectoryPoint>& points) {
  if (points.size() < 1) {
    return;
  }
  trajectory_points_.insert(trajectory_points_.begin(), points.begin(),
                            points.end());

  // TEST
  LOG_INFO("send out trajectory infos: ");
  for (std::size_t i = 0; i < 30 && i < trajectory_points_.size(); i += 1) {
    const auto& tp = trajectory_points_[i];
    LOG_DEBUG(
        "i, relative_time, x, y, theta, kappa, dkappa, "
        "s, v, a, j: {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, "
        "{:.3f}, "
        "{:.3f}, {:.3f}, {:.3f}",
        i, tp.relative_time(), tp.x(), tp.y(), tp.theta(), tp.kappa(),
        tp.dkappa(), tp.s(), tp.velocity(), tp.acceleration(), tp.jerk());
  }
}

bool DiscretizedTrajectory::is_valid() const {
  for (const auto& p : trajectory_points()) {
    double t = p.relative_time();
    double lon_v = p.velocity();
    if (!within_range(lon_v, FLAGS_planning_speed_lower_bound,
                      FLAGS_planning_speed_upper_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Velocity at relative time {} exceeds bound, value: {}, bound "
          "[{}, {}]",
          t, lon_v, FLAGS_planning_speed_lower_bound,
          FLAGS_planning_speed_upper_bound);
      return false;
    }

    double lon_a = p.acceleration();
    if (!within_range(lon_a,
                      FLAGS_planning_longitudinal_acceleration_lower_bound,
                      FLAGS_planning_longitudinal_acceleration_upper_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Longitudinal acceleration at relative time {} exceeds bound, "
          "value: {}, bound [{}, {}]",
          t, lon_a, FLAGS_planning_longitudinal_acceleration_lower_bound,
          FLAGS_planning_longitudinal_acceleration_upper_bound);
      return false;
    }

    double lat_a = lon_v * lon_v * p.kappa();
    if (!within_range(lat_a, -FLAGS_planning_lateral_acceleration_bound,
                      FLAGS_planning_lateral_acceleration_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Lateral acceleration at relative time {} exceeds bound, "
          "value: {}, bound [{}, {}]",
          t, lat_a, -FLAGS_planning_lateral_acceleration_bound,
          FLAGS_planning_lateral_acceleration_bound);
      return false;
    }

    double kappa = p.kappa();
    if (!within_range(kappa, -FLAGS_planning_kappa_bound,
                      FLAGS_planning_kappa_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Kappa at relative time {} exceeds bound, value: {}, bound "
          "[{}, {}]",
          t, kappa, -FLAGS_planning_kappa_bound, FLAGS_planning_kappa_bound);
      return false;
    }
  }

  for (std::size_t i = 1; i < num_of_points(); ++i) {
    TrajectoryPoint p0;
    TrajectoryPoint p1;
    if (!trajectory_point_at(i - 1, p0) || !trajectory_point_at(i, p1)) {
      LOG_ERROR("trajectory_point_at failed, set valid false");
      return false;
    }
    double t = p0.relative_time();

    double dt = p1.relative_time() - p0.relative_time();
    double d_lon_a = p1.acceleration() - p0.acceleration();
    double lon_jerk = d_lon_a / dt;
    if (!within_range(d_lon_a / dt,
                      FLAGS_planning_longitudinal_jerk_lower_bound,
                      FLAGS_planning_longitudinal_jerk_upper_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Longitudinal jerk at relative time {} exceeds bound, value: "
          "{}, bound [{}, {}]",
          t, lon_jerk, FLAGS_planning_longitudinal_jerk_lower_bound,
          FLAGS_planning_longitudinal_jerk_upper_bound);
      return false;
    }

    double d_lat_a = p1.velocity() * p1.velocity() * p1.kappa() -
                     p0.velocity() * p0.velocity() * p0.kappa();
    double lat_jerk = d_lat_a / dt;
    if (!within_range(lat_jerk, -FLAGS_planning_lateral_jerk_bound,
                      FLAGS_planning_lateral_jerk_bound,
                      FLAGS_planning_sanity_check_epsilon)) {
      LOG_WARN(
          "Lateral jerk at relative time {} exceeds bound, value: {}, "
          "bound [{}, {}]",
          t, lat_jerk, -FLAGS_planning_lateral_jerk_bound,
          FLAGS_planning_lateral_jerk_bound);
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
