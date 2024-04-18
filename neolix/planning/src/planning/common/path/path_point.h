#pragma once

#include <sstream>

#include "src/planning/common/data_center/path_context.h"
#include "common/math/vec2d.h"
#include "src/planning/public/planning_lib_header.h"
namespace neodrive {
namespace planning {

class PathPoint : public Vec2d {
 public:
  PathPoint() = default;
  ~PathPoint() = default;
  PathPoint(const Vec2d& coordinate, const double theta, const double kappa,
            const double dkappa, const double ddkappa, const double s);

  Vec2d coordinate() const;
  double theta() const;
  double kappa() const;
  double dkappa() const;
  double ddkappa() const;
  double s() const;
  PieceBoundary point_boundary() const;

  PathPoint* mutable_path_point();

  void set_theta(const double heading);
  void set_kappa(const double kappa);
  void set_dkappa(const double dkappa);
  void set_ddkappa(const double ddkappa);
  void set_s(const double s);
  void set_point_boundary(const PieceBoundary& point_boudanry);

 public:
  static bool interpolate(const PathPoint& p0, const PathPoint& p1,
                          const double s, PathPoint& pt);

  static bool interpolate_linear_approximation(const PathPoint& left,
                                               const PathPoint& right,
                                               const double s, PathPoint& pt);

 protected:
  // heading direction of the point
  double theta_ = 0.0;
  // derivative of heading with respect to s
  double kappa_ = 0.0;
  // second order derivative of heading
  double dkappa_ = 0.0;
  // third order derivative of heading
  double ddkappa_ = 0.0;
  // accumulated s along the path
  double s_ = 0.0;

  // boundary info
  PieceBoundary point_boudanry_{};
};

}  // namespace planning
}  // namespace neodrive
