#pragma once

#include "planning.pb.h"
#include "src/planning/common/path/path_point.h"

namespace neodrive {
namespace planning {

class TrajectoryPoint : public PathPoint {
 public:
  TrajectoryPoint() = default;
  //   TrajectoryPoint(const PathPoint& path_point, const double velocity,
  //                   const double acceleration, const double relative_time);

  TrajectoryPoint(const PathPoint& path_point, const double velocity,
                  const double acceleration, const double jerk,
                  const double relative_time);

  //   explicit TrajectoryPoint(
  //       const neodrive::global::planning::ADCTrajectoryPoint& point);
  ~TrajectoryPoint() = default;

 public:
  void set_velocity(const double v);
  void set_acceleration(const double a);
  void set_jerk(const double jerk);
  void set_relative_time(const double relative_time);
  void set_confidence(const double confidence);
  void set_steer(const double steer);
  void set_direction(const int direction);
  void set_segment_index(const int segment_index);
  void set_path_point(const PathPoint& path_point);

  double velocity() const;
  double acceleration() const;
  double jerk() const;
  double relative_time() const;
  double confidence() const;
  double steer() const;
  int direction() const;
  int segment_index() const;

  const PathPoint& path_point() const;
  const bool has_path_point() const;

  void Clear();

 public:
  static TrajectoryPoint interpolate(const TrajectoryPoint& p0,
                                     const TrajectoryPoint& p1, const double t);
  /**
   * Interpolate the trajectory point using linear interpolation;
   * The method will approximate the intermediate trajectory points
   * when the discretized trajectory is dense enough.
   */
  static TrajectoryPoint interpolate_linear_approximation(
      const TrajectoryPoint& p0, const TrajectoryPoint& p1, const double t);

 private:
  double v_ = 0.0;
  double a_ = 0.0;
  double relative_time_ = 0.0;
  double confidence_ = 1.0;
  double steer_ = 0.0;
  int direction_ = 0;      // 0:forward;1:backward
  int segment_index_ = 0;  // segment
  PathPoint path_point_{};
  bool has_path_point_{false};
  double jerk_{0.0};
};

}  // namespace planning
}  // namespace neodrive
