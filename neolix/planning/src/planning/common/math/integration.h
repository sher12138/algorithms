#pragma once

#include <array>
#include <functional>
#include <vector>

#include "neolix_log.h"

namespace neodrive {
namespace planning {

class Integration {
 public:
  Integration() = default;

  static double simpson(const std::vector<double>& funv_vec, const double dx,
                        const std::size_t nsteps);

  static double trapezoidal(const std::vector<double>& funv_vec,
                            const double dx, const std::size_t nsteps);

  // Given a target function and integral lower and upper bound,
  // compute the integral approximation using 5th order Gauss-Legendre
  // integration. The target function must be a smooth function. Example: target
  // function: auto func = [](const double& x) {return x * x;};
  //                  double integral = gauss_legendre(func, -2, 3);
  // This gives you the approximated integral of function x^2 in bound [-2, 3]
  static double gauss_legendre(const std::function<double(double)>& func,
                               const double lower_bound,
                               const double upper_bound);
};

}  // namespace planning
}  // namespace neodrive
