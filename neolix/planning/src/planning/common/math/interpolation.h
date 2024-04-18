#pragma once

#include <array>
#include <cmath>

#include "hermite_spline.h"
#include "neolix_log.h"
#include "common/math/math_utils.h"

namespace neodrive {
namespace planning {

class Interpolation {
 public:
  Interpolation() = delete;

  static double slerp(const double a0, const double p0, const double a1,
                      const double p1, const double p);

  template <typename T>
  static T lerp(const T& x0, const double p0, const T& x1, const double p1,
                const double p);

  template <typename T, std::size_t N>
  static std::array<T, N> hermite(const std::array<T, N>& x0, const double p0,
                                  const std::array<T, N>& x1, const double p1,
                                  const double p);

  static bool scatter_first_order_linear_interpolate(
      std::pair<double, double>& start_point,
      std::pair<double, double>& end_point,
      std::vector<std::pair<double, double>>* new_points,
      const double resolution = 0.2);
};

template <typename T>
inline T Interpolation::lerp(const T& x0, const double p0, const T& x1,
                             const double p1, const double p) {
  double r = 0.0;
  if (p0 + 1.0e-3 < p1) {
    r = (p - p0) / (p1 - p0);
    return x0 + (x1 - x0) * r;
  } else if (p0 > p1 + 1.0e-3) {
    r = (p - p1) / (p0 - p1);
    return x1 + (x0 - x1) * r;
  } else {
    return x0;
  }
}

inline double Interpolation::slerp(const double a0, const double p0,
                                   const double a1, const double p1,
                                   const double p) {
  if (std::abs(p1 - p0) <= kMathEpsilon) {
    return normalize_angle(a0);
  }
  double a0_n = normalize_angle(a0);
  double a1_n = normalize_angle(a1);

  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2.0 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2.0 * M_PI;
  }

  double r = (p - p0) / (p1 - p0);
  double a = a0_n + d * r;
  return normalize_angle(a);
}

template <typename T, std::size_t N>
inline std::array<T, N> Interpolation::hermite(const std::array<T, N>& x0,
                                               const double p0,
                                               const std::array<T, N>& x1,
                                               const double p1,
                                               const double p) {
  std::array<T, N> x;
  if (!(N == 2 || N == 3)) {
    LOG_ERROR(
        "Error: currently hermite interpolation only supports cubic and "
        "quintic!");
    return x;
  }

  HermiteSpline<T, 2 * N - 1> hermite_spline(x0, x1, p0, p1);

  for (std::size_t i = 0; i < N; ++i) {
    x[i] = hermite_spline.evaluate(i, p);
  }
  return x;
}

inline bool Interpolation::scatter_first_order_linear_interpolate(
    std::pair<double, double>& start_point,
    std::pair<double, double>& end_point,
    std::vector<std::pair<double, double>>* new_points,
    const double resolution) {
  if (new_points == nullptr) return false;
  double orin_x = start_point.first;
  double orin_y = start_point.second;

  double new_end_x = end_point.first - orin_x;
  double new_end_y = end_point.second - orin_y;
  // double set_orin_x = 0.0;
  // double set_orin_y = 0.0;
  new_points->clear();
  if (std::fabs(new_end_x) <= 2 * resolution &&
      std::fabs(new_end_y) <= 2 * resolution) {
    new_points->push_back(start_point);
    new_points->push_back(end_point);
    return true;
  }
  if (std::fabs(new_end_x) < 2 * resolution) {
    int count = std::floor(std::fabs(new_end_y) / resolution);
    for (int i = 0; i < count; ++i) {
      double tmp_y = (new_end_y) / count * i;
      double tmp_x = (new_end_x) / count * i;
      new_points->push_back(std::make_pair(tmp_x + orin_x, tmp_y + orin_y));
    }
    return true;
  }
  int count = std::floor(std::fabs(new_end_x) / resolution);
  for (int i = 0; i < count; ++i) {
    double tmp_y = (new_end_y) / count * i;
    double tmp_x = (new_end_x) / count * i;
    new_points->push_back(std::make_pair(tmp_x + orin_x, tmp_y + orin_y));
  }
  return true;
}

}  // namespace planning
}  // namespace neodrive
