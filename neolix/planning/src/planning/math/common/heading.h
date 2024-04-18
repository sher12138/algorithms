#pragma once

#include <cmath>

#include "common/math/math_utils.h"
#include "euler_angles_zxy.h"

namespace neodrive {
namespace planning {

// Returns heading (in radians) in [-PI, PI).
inline double quaternion_to_heading(double qw, double qx, double qy,
                                    double qz) {
  EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
  // Needs adding pi/2 to yaw because when the vehicle reference frame is RFU,
  // yaw is zero when the car is pointing to north.
  return normalize_angle(euler_angles.yaw + M_PI_2);
}

// Similar to above but takes a Quaternion object.
template <typename T>
inline double quaternion_to_heading(const Eigen::Quaternion<T> &q) {
  return quaternion_to_heading(q.w(), q.x(), q.y(), q.z());
}

// Returns a quaternion with zero roll, zero pitch, and the specified heading.
template <typename T>
inline Eigen::Quaternion<T> heading_to_quaternion(double heading) {
  // Needs deducting pi/2 from heading because the vehicle reference frame is
  // RFU.
  EulerAnglesZXY<T> euler_angles(heading - M_PI_2);
  return euler_angles.to_quaternion();
}

}  // namespace planning
}  // namespace neodrive
