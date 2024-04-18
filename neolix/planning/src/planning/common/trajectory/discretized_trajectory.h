#pragma once

#include <algorithm>
#include <limits>

#include "common/math/vec2d.h"
#include "trajectory_point.h"

namespace neodrive {
namespace planning {

class DiscretizedTrajectory {
 public:
  DiscretizedTrajectory() = default;
  DiscretizedTrajectory(const std::vector<TrajectoryPoint>& trajectory_points);
  ~DiscretizedTrajectory() = default;

  double time_length() const;

  double spatial_length() const;

  bool evaluate(const double relative_time, TrajectoryPoint& pt) const;

  bool start_point(TrajectoryPoint& pt) const;

  bool end_point(TrajectoryPoint& pt) const;

  bool evaluate_linear_approximation(const double relative_time,
                                     TrajectoryPoint& pt) const;

  bool query_relative_time_lower_bound_index(const double relative_time,
                                             std::size_t& index) const;

  bool query_nearest_point(const Vec2d& position, std::size_t& index) const;

  void append_trajectory_point(const TrajectoryPoint& trajectory_point);

  bool trajectory_point_at(const std::size_t index, TrajectoryPoint& pt) const;

  std::size_t num_of_points() const;

  const std::vector<TrajectoryPoint>& trajectory_points() const;
  std::vector<TrajectoryPoint>* mutable_trajectory_points();

  void set_trajectory_points(const std::vector<TrajectoryPoint>& points);

  void insert_into_front_without_last_point(
      const std::vector<TrajectoryPoint>& points);

  bool is_valid() const;

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
};

}  // namespace planning
}  // namespace neodrive
