#pragma once
#include <memory>
#include "common/coordinate/coordinate_math.h"
#include "common_geometry.pb.h"
#include "planning.pb.h"
namespace neodrive {
namespace common {
using neodrive::global::common::Point3D;
using neodrive::global::planning::ADCTrajectory;
using neodrive::global::planning::ADCTrajectoryPoint;

inline double DistanceCalculate(const ADCTrajectory &trajectory, int end_idx,
                                const int start_idx = 0) {
  if (start_idx > trajectory.adc_trajectory_point_size() - 1) {
    return 0;
  }
  if (end_idx >= trajectory.adc_trajectory_point_size())
    end_idx = trajectory.adc_trajectory_point_size() - 1;
  double ret = 0.0;
  for (int i = start_idx + 1; i <= end_idx; ++i) {
    ret += Distance2d(trajectory.adc_trajectory_point(i - 1),
                      trajectory.adc_trajectory_point(i));
  }
  return ret;
}
}  // namespace common
}  // namespace neodrive