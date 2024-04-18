#include "sl_point.h"

namespace neodrive {
namespace planning_rl {

SLPoint::SLPoint(const double s, const double l) : Vec2d(s, l) {}

double SLPoint::s() const { return x_; }

double SLPoint::l() const { return y_; }

void SLPoint::set_s(const double s) { set_x(s); }

void SLPoint::set_l(const double l) { set_y(l); }

SLPoint SLPoint::interpolate(const SLPoint& start, const SLPoint& end,
                             const double weight) {
  double s = start.s() * (1 - weight) + end.s() * weight;
  double l = start.l() * (1 - weight) + end.l() * weight;
  return SLPoint(s, l);
}

Json::Value SLPoint::to_json() const {
  Json::Value root;
  root["s"] = std::to_string(s());
  root["l"] = std::to_string(l());
  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
