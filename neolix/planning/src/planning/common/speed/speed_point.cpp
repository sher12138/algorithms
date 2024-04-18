#include "speed_point.h"

namespace neodrive {
namespace planning {

SpeedPoint::SpeedPoint(const STPoint& st_point, const double v, const double a,
                       const double j)
    : STPoint(st_point), v_(v), a_(a), j_(j) {}

SpeedPoint::SpeedPoint(const double s, const double t, const double v,
                       const double a, const double j)
    : STPoint(s, t), v_(v), a_(a), j_(j) {}

void SpeedPoint::set_v(const double v) { v_ = v; }

void SpeedPoint::set_a(const double a) { a_ = a; }

void SpeedPoint::set_j(const double j) { j_ = j; }

double SpeedPoint::v() const { return v_; }

double SpeedPoint::a() const { return a_; }

double SpeedPoint::j() const { return j_; }

SpeedPoint SpeedPoint::interpolate(const SpeedPoint& left,
                                   const SpeedPoint& right,
                                   const double weight) {
  double s = (1. - weight) * left.s() + weight * right.s();
  double t = (1. - weight) * left.t() + weight * right.t();
  double v = (1. - weight) * left.v() + weight * right.v();
  double a = (1. - weight) * left.a() + weight * right.a();
  double j = (1. - weight) * left.j() + weight * right.j();
  return SpeedPoint(s, t, v, a, j);
}

}  // namespace planning
}  // namespace neodrive
