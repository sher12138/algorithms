#pragma once

#include <math.h>

#include <Eigen/Eigen>

namespace neodrive {
namespace planning {
namespace sim_planner {

template <typename T>
int sgn(const T val) {
  return (T(0) < val) - (val < T(0));
}

long long fac(int n);

long long nchoosek(int n, int k);

double normalizeAngle(const double theta);

Eigen::Vector2d rotateVector2d(const Eigen::Vector2d v, const double angle);

double convertVec2dToAngle(const Eigen::Vector2d v);

double truncate(const double val_in, const double lower, const double upper);

double normalizeWithBound(const double val_in, const double lower,
                          const double upper, const double new_lower,
                          const double new_upper);

bool remapUsingQuadraticFuncAroundSmallValue(const double th,
                                             const double val_in,
                                             double* val_out);

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
