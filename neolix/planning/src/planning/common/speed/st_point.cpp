#include "st_point.h"

namespace neodrive {
namespace planning {

STPoint::STPoint(const double s, const double t) : Vec2d(t, s){};

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { return set_y(s); }

void STPoint::set_t(const double t) { return set_x(t); }

}  // namespace planning
}  // namespace neodrive
