#include "calculations.h"

#include "src/planning/common/planning_macros.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

const double kPi = acos(-1.0);

long long fac(int n) {
  if (n == 0) return 1;
  if (n == 1) return 1;
  if (n == 2) return 2;
  if (n == 3) return 6;
  if (n == 4) return 24;
  if (n == 5) return 120;

  long long ans = 1;
  for (int i = 1; i <= n; i++) ans *= i;
  return ans;
}

long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

double normalizeAngle(const double theta) {
  double theta_tmp = theta;
  theta_tmp -= (theta >= kPi) * 2 * kPi;
  theta_tmp += (theta < -kPi) * 2 * kPi;
  return theta_tmp;
}

Eigen::Vector2d rotateVector2d(const Eigen::Vector2d v, const double angle) {
  return Eigen::Vector2d(v[0] * cos(angle) - v[1] * sin(angle),
                         v[0] * sin(angle) + v[1] * cos(angle));
}

double convertVec2dToAngle(const Eigen::Vector2d v) {
  return atan2(v[1], v[0]);
}

double truncate(const double val_in, const double lower, const double upper) {
  if (lower > upper) {
    LOG_ERROR("[Calculations]Invalid input!");
    assert(false);
  }
  double res = val_in;
  res = std::max(res, lower);
  res = std::min(res, upper);
  return res;
}

double normalizeWithBound(const double val_in, const double lower,
                          const double upper, const double new_lower,
                          const double new_upper) {
  if (new_lower > new_upper) {
    LOG_ERROR("[Calculations]Invalid input!");
    assert(false);
  }
  double val_bounded = truncate(val_in, lower, upper);
  double ratio = (val_bounded - lower) / (upper - lower);
  double res = new_lower + (new_upper - new_lower) * ratio;
  return res;
}

bool remapUsingQuadraticFuncAroundSmallValue(const double th,
                                             const double val_in,
                                             double* val_out) {
  double c = 1.0 / th;
  if (fabs(val_in) <= fabs(th)) {
    // quadratic, y = c * x ^ 2
    *val_out = sgn(val_in) * c * val_in * val_in;
  } else {
    // linear, y = x
    *val_out = val_in;
  }
  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
