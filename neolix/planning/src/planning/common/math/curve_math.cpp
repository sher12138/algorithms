#include "curve_math.h"

namespace neodrive {
namespace planning {

double CurveMath::compute_curvature(const double dx, const double d2x,
                                    const double dy, const double d2y) {
  double a = dx * d2y - dy * d2x;
  double dx_dy_norm_square = dx * dx + dy * dy;
  double dx_dy_norm = std::sqrt(dx_dy_norm_square);
  double b = dx_dy_norm_square * dx_dy_norm;
  return a / b;
}

double CurveMath::compute_curvature_derivative(
    const double dx, const double d2x, const double d3x, const double dy,
    const double d2y, const double d3y) {
  const double a = dx * d2y - dy * d2x;
  const double b = dx * d3y - dy * d3x;
  const double c = dx * d2x + dy * d2y;
  const double d = dx * dx + dy * dy;

  return (b * d - 3.0 * a * c) / std::pow(d, 3.0);
}

void CurveMath::compute_curvature_max_speed(const double curv,
                                            double* max_speed) {
  if (fabs(curv) < 0.1) return;
  // linear function 0.1-2 m/s; 0.2-1.5 m/s
  double tmp_speed = -5 * fabs(curv) + 2.5;
  tmp_speed = fmax(1.5, tmp_speed);
  *max_speed = tmp_speed;

  return;
}

}  // namespace planning
}  // namespace neodrive
