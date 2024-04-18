#pragma once

#include "vec2d.h"

namespace neodrive {
namespace planning {

template <typename T>
inline double cross_prod(const T& point1, const T& point2, const T& point3) {
  return (point2.x() - point1.x()) * (point3.y() - point1.y()) -
         (point3.x() - point1.x()) * (point2.y() - point1.y());
}

template <typename T>
inline double inner_prod(const T& point1, const T& point2, const T& point3) {
  return (point2.x() - point1.x()) * (point3.x() - point1.x()) +
         (point2.y() - point1.y()) * (point3.y() - point1.y());
}

// Wrap angle to [0, 2 * PI).
inline double wrap_angle(const double angle) {
  const double new_angle = fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

// Normalize angle to [-PI, PI).
inline double normalize_angle(const double angle) {
  const double new_angle = fmod(angle + M_PI, M_PI * 2.0);
  return (new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle) - M_PI;
}

// The difference between from and to. The range is between [-PI, PI).
inline double angle_diff(const double from, const double to) {
  return normalize_angle(to - from);
}

template <typename T>
inline T sqr(const T value) {
  return value * value;
}

// Cartesian coordinates to Polar coordinates
inline std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

/**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

/**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

bool IsDoubleEqual(const double a, const double b, const double precision);

}  // namespace planning
}  // namespace neodrive
