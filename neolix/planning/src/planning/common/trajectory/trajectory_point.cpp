#include "trajectory_point.h"

#include "common/math/hermite_spline.h"
#include "common/math/integration.h"
#include "common/math/interpolation.h"

namespace neodrive {
namespace planning {
// using neodrive::global::planning::ADCTrajectoryPoint;

// TrajectoryPoint::TrajectoryPoint(const ADCTrajectoryPoint& point)
//     : PathPoint(Vec2d(point.x(), point.y()), point.theta(),
//     point.curvature(),
//                 point.curvature_change_rate(), 0.0, point.accumulated_s()),
//       v_(point.speed()),
//       a_(point.acceleration_s()),
//       relative_time_(point.relative_time()),
//       path_point_(PathPoint(Vec2d(point.x(), point.y()), point.theta(),
//                             point.curvature(), point.curvature_change_rate(),
//                             0.0, point.accumulated_s())) {}

// TrajectoryPoint::TrajectoryPoint(const PathPoint& path_point,
//                                  const double velocity,
//                                  const double acceleration,
//                                  const double relative_time)
//     : PathPoint(path_point),
//       v_(velocity),
//       a_(acceleration),
//       relative_time_(relative_time),
//       path_point_(path_point) {}

TrajectoryPoint::TrajectoryPoint(const PathPoint& path_point,
                                 const double velocity,
                                 const double acceleration, const double jerk,
                                 const double relative_time)
    : PathPoint(path_point),
      v_(velocity),
      a_(acceleration),
      jerk_(jerk),
      relative_time_(relative_time),
      path_point_(path_point) {
  set_path_point(path_point);
}

void TrajectoryPoint::Clear() {
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  kappa_ = 0.0;
  dkappa_ = 0.0;
  s_ = 0.0;
  v_ = 0.0;
  a_ = 0.0;
  jerk_ = 0.0;
  relative_time_ = 0.0;
  confidence_ = 1.0;
  steer_ = 0.0;
  direction_ = 0;
  segment_index_ = 0;
}

void TrajectoryPoint::set_velocity(const double v) { v_ = v; }

double TrajectoryPoint::velocity() const { return v_; }

void TrajectoryPoint::set_acceleration(const double a) { a_ = a; }

void TrajectoryPoint::set_jerk(const double jerk) { jerk_ = jerk; }

void TrajectoryPoint::set_confidence(const double confidence) {
  confidence_ = confidence;
}

double TrajectoryPoint::acceleration() const { return a_; }

double TrajectoryPoint::jerk() const { return jerk_; }

void TrajectoryPoint::set_relative_time(const double relative_time) {
  relative_time_ = relative_time;
}

double TrajectoryPoint::relative_time() const { return relative_time_; }

void TrajectoryPoint::set_path_point(const PathPoint& path_point) {
  path_point_ = path_point;
  has_path_point_ = true;
}

const bool TrajectoryPoint::has_path_point() const { return has_path_point_; }

const PathPoint& TrajectoryPoint::path_point() const { return path_point_; }

double TrajectoryPoint::confidence() const { return confidence_; }

void TrajectoryPoint::set_steer(const double steer) { steer_ = steer; }

double TrajectoryPoint::steer() const { return steer_; }

void TrajectoryPoint::set_direction(const int direction) {
  direction_ = direction;
}

int TrajectoryPoint::direction() const { return direction_; }

void TrajectoryPoint::set_segment_index(const int segment_index) {
  segment_index_ = segment_index;
}

int TrajectoryPoint::segment_index() const { return segment_index_; }

TrajectoryPoint TrajectoryPoint::interpolate(const TrajectoryPoint& p0,
                                             const TrajectoryPoint& p1,
                                             const double t) {
  if (std::abs(p0.s() - p1.s()) < 1.0e-4) {
    return p1;
  }

  double t0 = p0.relative_time();
  double t1 = p1.relative_time();

  std::array<double, 2> dx0{{p0.velocity(), p0.acceleration()}};
  std::array<double, 2> dx1{{p1.velocity(), p1.acceleration()}};
  HermiteSpline<double, 3> dynamic_spline(dx0, dx1, t0, t1);

  double s0 = 0.0;
  auto func_v = [&dynamic_spline](const double t) {
    return dynamic_spline.evaluate(0, t);
  };

  double s1 = Integration::gauss_legendre(func_v, t0, t1);
  double s = Integration::gauss_legendre(func_v, t0, t);

  if (std::abs(s0 - s1) < 1.0e-4) {
    return p1;
  }

  double v = dynamic_spline.evaluate(0, t);
  double a = dynamic_spline.evaluate(1, t);
  double j = dynamic_spline.evaluate(2, t);

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
  // linear interpolation for confidence
  double confidence =
      Interpolation::lerp(p0.confidence(), t0, p1.confidence(), t1, t);

  PathPoint pp({x, y}, theta, kappa, dkappa, ddkappa, s);

  // TODO(chi): set t
  TrajectoryPoint traj_point(pp, v, a, j, 0.0);
  traj_point.set_confidence(confidence);
  return traj_point;
}

TrajectoryPoint TrajectoryPoint::interpolate_linear_approximation(
    const TrajectoryPoint& p0, const TrajectoryPoint& p1, const double t) {
  double t0 = p0.relative_time();
  double t1 = p1.relative_time();

  TrajectoryPoint tp;
  double x = Interpolation::lerp(p0.x(), t0, p1.x(), t1, t);
  double y = Interpolation::lerp(p0.y(), t0, p1.y(), t1, t);
  double theta = Interpolation::slerp(p0.theta(), t0, p1.theta(), t1, t);
  double kappa = Interpolation::lerp(p0.kappa(), t0, p1.kappa(), t1, t);
  double dkappa = Interpolation::lerp(p0.dkappa(), t0, p1.dkappa(), t1, t);
  double ddkappa = Interpolation::lerp(p0.ddkappa(), t0, p1.ddkappa(), t1, t);
  double s = Interpolation::lerp(p0.s(), t0, p1.s(), t1, t);
  double velocity =
      Interpolation::lerp(p0.velocity(), t0, p1.velocity(), t1, t);
  double acceleration =
      Interpolation::lerp(p0.acceleration(), t0, p1.acceleration(), t1, t);
  double j = Interpolation::lerp(p0.jerk(), t0, p1.jerk(), t1, t);
  double confidence =
      Interpolation::lerp(p0.confidence(), t0, p1.confidence(), t1, t);

  PathPoint pp({x, y}, theta, kappa, dkappa, ddkappa, s);
  TrajectoryPoint traj_point(pp, velocity, acceleration, j, t);
  traj_point.set_confidence(confidence);
  return traj_point;
}

}  // namespace planning
}  // namespace neodrive
