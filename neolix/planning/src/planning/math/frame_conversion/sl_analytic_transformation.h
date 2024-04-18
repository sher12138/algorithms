/**
 * @file : sl_analytic_transformation.h
 * @brief: explicit analytic form of sl to x, y, theta, kappa transformation
 *
 *           x = x_ref - l * sin(theta)
 *           y = y_ref + l * cos(theta)
 *           theta = theta_ref + atan2(dl / ds, 1 - l * kappa_ref);
 *           To derive kappa, we need the formula below:
 *           kappa = (x' * y'' - y' * x'') / ((x')^2 + (y')^2)^(3/2)
 *           after some calculations, kappa is a expilicit form of theta, l and
 *its derivatives
 * @Notice: Theta's expicit form need first derivative of l and theta
 *            Kappa's explicit form need second derivative of l and theta
 **/

#pragma once

#include <cmath>
#include <cstddef>

#include "common/math/double.h"
#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

class SLAnalyticTransformation {
 public:
  // given sl point extract x, y, theta, kappa
  static double calculate_theta(const double theta_ref, const double kappa_ref,
                                const double l, const double dl);
  static double calculate_kappa(const double kappa_ref, const double dkappa_ref,
                                const double l, const double dl,
                                const double ddl);
  static Vec2d calculate_xypoint(const double theta_ref, const Vec2d& point_ref,
                                 const double l);
  /**
   *   @brief: given sl, theta, and road's theta, kappa, extract derivative l,
   *second order derivative l:
   *   @reference:  in paper: optimal trajectory generation for dynamic street
   *scenarios in a frenet frame
   **/
  static double calculate_lateral_derivative(const double theta_ref,
                                             const double theta, const double l,
                                             const double kappa_ref);

  // given sl, theta, and road's theta, kappa, extract second order derivative
  static double calculate_second_order_lateral_derivative(
      const double theta_ref, const double theta, const double kappa_ref,
      const double kappa, const double dkappa_ref, const double l);
};

}  // namespace planning
}  // namespace neodrive
