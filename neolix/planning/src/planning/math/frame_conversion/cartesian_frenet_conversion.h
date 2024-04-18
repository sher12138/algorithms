#pragma once

#include <array>
#include <cmath>

#include "common/math/math_utils.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

// Notations:
// s_condition = [s, s_dot, s_ddot]
// s: longitudinal coordinate w.r.t reference line.
// s_dot: ds / dt
// s_ddot: d(s_dot) / dt
// d_condition = [d, d_prime, d_pprime]
// d: lateral coordinate w.r.t. reference line
// d_prime: dd / ds
// d_pprime: d(d_prime) / ds
class CartesianFrenetConverter {
 public:
  /**
   * Convert a vehicle state in Cartesian frame to Frenet frame.
   * Decouple a 2d movement to two independent 1d movement w.r.t. reference
   * line. The lateral movement is a function of longitudinal accumulated
   * distance s to achieve better satisfaction of nonholonomic constraints.
   */
  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3>* const ptr_s_condition,
                                  std::array<double, 3>* const ptr_d_condition);
  /**
   * Convert a vehicle state in Frenet frame to Cartesian frame.
   * Combine two independent 1d movement w.r.t. reference line to a 2d movement.
   */
  static bool frenet_to_cartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3>& s_condition,
                                  const std::array<double, 3>& d_condition,
                                  double* const ptr_x, double* const ptr_y,
                                  double* const ptr_theta,
                                  double* const ptr_kappa, double* const ptr_v,
                                  double* const ptr_a);

  static void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double theta, const double kappa,
                                  double* ptr_s,
                                  std::array<double, 3>* const ptr_d_condition);

  static bool frenet_to_cartesian(
      const double rs, const double rx, const double ry, const double rtheta,
      const double rkappa, const double rdkappa, const double s,
      const std::array<double, 3>& d_condition, double* const ptr_x,
      double* const ptr_y, double* const ptr_theta, double* const ptr_kappa);
};

}  // namespace planning
}  // namespace neodrive
