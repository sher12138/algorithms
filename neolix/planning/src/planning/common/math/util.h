#pragma once

#include <cmath>

#include "common/math/math_utils.h"
#include "common/math/segment2d.h"
#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

template <typename T>
bool within_range(const T v, const T lower, const T upper, const T epsilon) {
  if (v < lower - epsilon || v > upper + epsilon) {
    return false;
  }
  return true;
}

template <typename T>
T clamp(const T v, const T lower, const T upper) {
  if (v < lower) {
    return lower;
  }
  if (v > upper) {
    return upper;
  }
  return v;
}
/*Coordiante transformation, x2y,y2(-x)*/
// x_earth: start pos;x_target:end pos; x_target_local:end pos in start ISO coor
bool earth2vehicle(const double x_earth, const double y_earth,
                   const double theta_earth, const double x_target,
                   const double y_target, const double theta_target,
                   double &x_target_local, double &y_target_local,
                   double &theta_target_local);

// x_earth: start pos; x_target_local: local coordinate to
// x_earth;x_target_earth: transfer to world coor
bool vehicle2earth(const double x_earth, const double y_earth,
                   const double theta_earth, const double x_target_local,
                   const double y_target_local, const double theta_target_local,
                   double &x_target_earth, double &y_target_earth,
                   double &theta_target_earth);

// collision check, true: collided
bool PolygonCollision(const std::vector<Vec2d> &self_bound,
                      const std::vector<Vec2d> &other_bound);

bool PointInPolygon(const double &x, const double &y,
                    const std::vector<Vec2d> &bound);

bool PointInPolygonINT(
    const long long &x, const long long &y,
    const std::vector<std::pair<long long, long long>> &bound);

bool VectorValidTest(const std::vector<Vec2d> &points);

bool LineIntersectWithPolygon(const Vec2d &pt1, const Vec2d &pt2,
                              const std::vector<Vec2d> &bound);

bool calc_left_right_dis_to_freespace(const std::vector<Vec2d> &free_space,
                                      const double &free_space_x_range,
                                      const double &free_space_y_range,
                                      const Vec2d &pt, const double &theta,
                                      double *left_dis, double *right_dis);

bool calc_min_dis_from_freespace_to_point(const std::vector<Vec2d> &free_space,
                                          const Vec2d &pt, double *min_dis);

bool calc_min_dis_from_freespace_to_segment(
    const std::vector<Vec2d> &free_space, const Vec2d &pt1, const Vec2d &pt2,
    double *min_dis);

/**
 * @param func The target single-variable function to minimize.
 * @param lower_bound The lower bound of the interval.
 * @param upper_bound The upper bound of the interval.
 * @param tol The tolerance of error.
 * @return The value that minimize the function fun.
 */
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

}  // namespace planning
}  // namespace neodrive
