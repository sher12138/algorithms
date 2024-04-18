#include "frenet_frame_point.h"

namespace neodrive {
namespace planning {

FrenetFramePoint::FrenetFramePoint(const double s, const double l,
                                   const double dl, const double ddl)
    : SLPoint(s, l), dl_(dl), ddl_(ddl) {}

void FrenetFramePoint::set_dl(const double dl) { dl_ = dl; }

void FrenetFramePoint::set_ddl(const double ddl) { ddl_ = ddl; }

void FrenetFramePoint::set_param(const double s, const double l,
                                 const double dl, const double ddl) {
  set_s(s);
  set_l(l);
  set_dl(dl);
  set_ddl(ddl);
}

void FrenetFramePoint::set_heading_diff(const double heading_diff) {
  heading_diff_ = heading_diff;
}

double FrenetFramePoint::dl() const { return dl_; }

double FrenetFramePoint::ddl() const { return ddl_; }

double FrenetFramePoint::heading_diff() const { return heading_diff_; }

}  // namespace planning
}  // namespace neodrive
