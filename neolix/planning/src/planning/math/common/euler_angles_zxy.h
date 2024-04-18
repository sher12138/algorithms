#pragma once

#include <cmath>

#include "common/math/math_utils.h"

namespace neodrive {
namespace planning {

template <typename T>
struct EulerAnglesZXY {
  T roll, pitch, yaw;

  // Constructs an identity rotation.
  EulerAnglesZXY() : roll(0), pitch(0), yaw(0) {}

  // Constructs a rotation using only yaw.
  explicit EulerAnglesZXY(T yaw) : roll(0), pitch(0), yaw(yaw) {}

  // Constructs a rotation using roll, pitch, and yaw.
  EulerAnglesZXY(T roll, T pitch, T yaw) : roll(roll), pitch(pitch), yaw(yaw) {}

  // Constructs a rotation using components of a quaternion.
  EulerAnglesZXY(T qw, T qx, T qy, T qz)
      : roll(atan2(2.0 * (qw * qy - qx * qz),
                   2.0 * (sqr<T>(qw) + sqr<T>(qz)) - 1.0)),
        pitch(asin(2.0 * (qw * qx + qy * qz))),
        yaw(atan2(2.0 * (qw * qz - qx * qy),
                  2.0 * (sqr<T>(qw) + sqr<T>(qy)) - 1.0)) {}

  // Constructs a rotation using a quaternion.
  explicit EulerAnglesZXY(const Eigen::Quaternion<T>& q)
      : EulerAnglesZXY(q.w(), q.x(), q.y(), q.z()) {}

  // Normalizes roll, pitch, and yaw to [-PI, PI).
  void normalize() {
    roll = normalize_angle(roll);
    pitch = normalize_angle(pitch);
    yaw = normalize_angle(yaw);
  }

  // Check if the rotation is valid. A valid rotation must have -PI/2 < pitch <
  // PI/2.
  bool is_valid() {
    normalize();
    return pitch < M_PI_2 && pitch > -M_PI_2;
  }

  // Converts to a quaternion. The scalar part is guarantee to be non-negative.
  Eigen::Quaternion<T> to_quaternion() const {
    // Uses double for internal calculation for better precision.
    double r = roll * 0.5;
    double p = pitch * 0.5;
    double y = yaw * 0.5;

    double sr = sin(r);
    double sp = sin(p);
    double sy = sin(y);

    double cr = cos(r);
    double cp = cos(p);
    double cy = cos(y);

    T qw = cr * cp * cy - sr * sp * sy;
    T qx = cr * sp * cy - sr * cp * sy;
    T qy = cr * sp * sy + sr * cp * cy;
    T qz = cr * cp * sy + sr * sp * cy;
    if (qw < 0.0) return {-qw, -qx, -qy, -qz};
    return {qw, qx, qy, qz};
  }
};

using EulerAnglesZXYf = EulerAnglesZXY<float>;
using EulerAnglesZXYd = EulerAnglesZXY<double>;

}  // namespace planning
}  // namespace neodrive
