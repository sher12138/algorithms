#pragma once
#include "common/coordinate/coodrdinate_convertion.h"
#include "vec2d.h"
#include "vec3d.h"

namespace neodrive {
namespace planning {
namespace {
template <typename T>
static void Fiting(const double start_x, const double start_y,
                   const double start_yaw, const double end_x,
                   const double end_y, const double end_yaw,
                   std::vector<T> &ret, const double step = 0.01) {
  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 1> B;
  A << pow(start_x, 5), pow(start_x, 4), pow(start_x, 3), pow(start_x, 2),
      start_x, 1, pow(end_x, 5), pow(end_x, 4), pow(end_x, 3), pow(end_x, 2),
      end_x, 1, 5 * pow(start_x, 4), 4 * pow(start_x, 3), 3 * pow(start_x, 2),
      2 * start_x, 1, 0, 5 * pow(end_x, 4), 4 * pow(end_x, 3),
      3 * pow(end_x, 2), 2 * end_x, 1, 0, 20 * pow(start_x, 3),
      12 * pow(start_x, 2), 6 * start_x, 2, 0, 0, 20 * pow(end_x, 3),
      12 * pow(end_x, 2), 6 * end_x, 2, 0, 0;

  B << start_y, end_y, std::tan(start_yaw), std::tan(end_yaw), 0.0, 0.0;

  Eigen::Matrix<double, 6, 1> X = A.colPivHouseholderQr().solve(B);
  LOG_INFO("coefs: {:.9f}, {:.9f},{:.9f}, {:.9f},{:.9f}, {:.9f}", X(0), X(1),
           X(2), X(3), X(4), X(5));
  bool is_reverse = end_x < start_x;
  for (double x = start_x; is_reverse ? x >= end_x : x <= end_x;
       is_reverse ? x -= step : x += step) {
    double y{X(0) * pow(x, 5) + X(1) * pow(x, 4) + X(2) * pow(x, 3) +
             X(3) * pow(x, 2) + X(4) * x + X(5)};
    double theta{std::atan2(5 * X(0) * pow(x, 4) + 4 * X(1) * pow(x, 3) +
                                3 * X(2) * pow(x, 2) + 2 * X(3) * x + X(4),
                            1.)};
    if (!ret.empty() && theta > ret.back().z() + M_PI_2) {
      theta -= M_PI;
    }
    if (!ret.empty() && theta < ret.back().z() - M_PI_2) {
      theta += M_PI;
    }
    ret.emplace_back(x, y, theta);
  }
}
}  // namespace

template <typename T>
static void QuinticCurveFiting(const double start_x, const double start_y,
                               const double start_yaw, const double end_x,
                               const double end_y, const double end_yaw,
                               std::vector<T> &ret, const double step = 0.01,
                               bool need_more_pts = false) {
  if (Vec2d{start_x, start_y}.distance_to(Vec2d{end_x, end_y}) <= step ||
      (std::abs(end_y - start_y) < step && std::abs(end_x - start_x) < step)) {
    ret.emplace_back(start_x, start_y, start_yaw);
    ret.emplace_back(end_x, end_y, end_yaw);
  } else if (need_more_pts || M_PI_2 == start_yaw || M_PI_2 == end_yaw ||
             -M_PI_2 == start_yaw || -M_PI_2 == end_yaw ||
             std::abs(end_y - start_y) > std::abs(end_x - start_x)) {
    double rotate_yaw = std::atan2(end_y - start_y, end_x - start_x);
    Vec3d start_vec{start_x, start_y, start_yaw}, start_ralative;
    Vec3d end_vec{end_x, end_y, end_yaw}, end_relative;
    Vec3d rotate_vec{0., 0., rotate_yaw};
    common::ConvertToRelativeCoordinate(start_vec, rotate_vec, start_ralative);
    common::ConvertToRelativeCoordinate(end_vec, rotate_vec, end_relative);
    Fiting(start_ralative.x(), start_ralative.y(), start_ralative.z(),
           end_relative.x(), end_relative.y(), end_relative.z(), ret, step);
    for (auto &each_pt : ret) {
      common::ConvertToWorldCoordinate(each_pt, rotate_vec, each_pt);
    }
  } else {
    Fiting(start_x, start_y, start_yaw, end_x, end_y, end_yaw, ret, step);
  }
}

template <typename T>
static void QuinticCurveFiting(Vec3d start, Vec3d end, std::vector<T> &ret,
                               const double step = 0.01,
                               bool need_more_pts = false) {
  QuinticCurveFiting(start.x(), start.y(), start.z(), end.x(), end.y(), end.z(),
                     ret, step, need_more_pts);
}

}  // namespace planning
}  // namespace neodrive
