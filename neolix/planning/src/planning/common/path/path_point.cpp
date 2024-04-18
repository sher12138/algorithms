#include "path_point.h"

#include "common/math/hermite_spline.h"
#include "common/math/integration.h"

namespace neodrive {
namespace planning {

PathPoint::PathPoint(const Vec2d& point, const double heading,
                     const double kappa, const double dkappa,
                     const double ddkappa, const double s)
    : Vec2d(point),
      theta_(heading),
      kappa_(kappa),
      dkappa_(dkappa),
      ddkappa_(ddkappa),
      s_(s) {}

Vec2d PathPoint::coordinate() const { return Vec2d(x(), y()); }

double PathPoint::theta() const { return theta_; }

double PathPoint::kappa() const { return kappa_; }

double PathPoint::dkappa() const { return dkappa_; }

double PathPoint::ddkappa() const { return ddkappa_; }

double PathPoint::s() const { return s_; }

PieceBoundary PathPoint::point_boundary() const { return point_boudanry_; }

PathPoint* PathPoint::mutable_path_point() { return this; }

void PathPoint::set_theta(const double heading) { theta_ = heading; }

void PathPoint::set_kappa(const double kappa) { kappa_ = kappa; }

void PathPoint::set_dkappa(const double dkappa) { dkappa_ = dkappa; }

void PathPoint::set_ddkappa(const double ddkappa) { ddkappa_ = ddkappa; }

void PathPoint::set_s(const double s) { s_ = s; }

void PathPoint::set_point_boundary(const PieceBoundary& point_boudanry) {
  point_boudanry_ = point_boudanry;
}

bool PathPoint::interpolate(const PathPoint& p0, const PathPoint& p1,
                            const double s, PathPoint& pt) {
  double s0 = p0.s_;
  double s1 = p1.s_;
  if (!(s0 <= s && s <= s1)) return false;

  std::array<double, 2> gx0{{p0.theta(), p0.kappa()}};
  std::array<double, 2> gx1{{p1.theta(), p1.kappa()}};
  HermiteSpline<double, 3> geometry_spline(gx0, gx1, s0, s1);

  auto func_cos_theta = [&geometry_spline](const double s) {
    auto theta = geometry_spline.evaluate(0, s);
    return std::cos(theta);
  };

  auto func_sin_theta = [&geometry_spline](const double s) {
    auto theta = geometry_spline.evaluate(0, s);
    return std::sin(theta);
  };

  double x = p0.x() + Integration::gauss_legendre(func_cos_theta, s0, s);
  double y = p0.y() + Integration::gauss_legendre(func_sin_theta, s0, s);
  double theta = geometry_spline.evaluate(0, s);
  double kappa = geometry_spline.evaluate(1, s);
  double dkappa = geometry_spline.evaluate(2, s);
  double ddkappa = geometry_spline.evaluate(3, s);

  PathPoint tmp_pt({x, y}, theta, kappa, dkappa, ddkappa, s);
  pt = tmp_pt;
  return true;
}

bool PathPoint::interpolate_linear_approximation(const PathPoint& left,
                                                 const PathPoint& right,
                                                 const double s,
                                                 PathPoint& pt) {
  double s0 = left.s();
  double s1 = right.s();
  if (s0 >= s1) return false;

  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * left.x() + weight * right.x();
  double y = (1 - weight) * left.y() + weight * right.y();
  double cos_heading =
      (1 - weight) * std::cos(left.theta()) + weight * std::cos(right.theta());
  double sin_heading =
      (1 - weight) * std::sin(left.theta()) + weight * std::sin(right.theta());
  double heading = std::atan2(sin_heading, cos_heading);
  double kappa = (1 - weight) * left.kappa() + weight * right.kappa();
  double dkappa = (1 - weight) * left.dkappa() + weight * right.dkappa();
  double ddkappa = (1 - weight) * left.ddkappa() + weight * right.ddkappa();

  PathPoint tmp_pt({x, y}, heading, kappa, dkappa, ddkappa, s);
  pt = tmp_pt;
  return true;
}

}  // namespace planning
}  // namespace neodrive
