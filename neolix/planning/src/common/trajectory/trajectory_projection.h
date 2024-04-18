#pragma once
#include <memory>
#include "common/coordinate/coordinate_math.h"
#include "common_geometry.pb.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
namespace neodrive {
namespace common {
using neodrive::global::common::Point3D;
using neodrive::global::perception::Point;
using neodrive::global::planning::ADCTrajectory;
using neodrive::global::planning::ADCTrajectoryPoint;

template <typename T>
static inline int TrajectoryProjection(const T& pt,
                                       const ADCTrajectory& trajectory,
                                       double& dist_to_point) {
  if (trajectory.adc_trajectory_point().empty()) return -1;
  if (trajectory.adc_trajectory_point_size() == 1) {
    dist_to_point = Distance2d(pt, trajectory.adc_trajectory_point(0));
    return 0;
  }
  double pre_dist = Distance2d(pt, trajectory.adc_trajectory_point(0));
  double curr_dist = 0.0;
  for (int i = 1; i < trajectory.adc_trajectory_point_size(); ++i) {
    curr_dist = Distance2d(pt, trajectory.adc_trajectory_point(i));
    if (curr_dist > pre_dist) {
      dist_to_point = pre_dist;
      return i - 1;
    }
    pre_dist = curr_dist;
  }
  dist_to_point = curr_dist;
  return trajectory.adc_trajectory_point_size() - 1;
}
}  // namespace common
}  // namespace neodrive
