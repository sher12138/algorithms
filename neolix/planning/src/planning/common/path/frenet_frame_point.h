#pragma once

#include "sl_point.h"

namespace neodrive {
namespace planning {

class FrenetFramePoint : public SLPoint {
 public:
  FrenetFramePoint() = default;
  ~FrenetFramePoint() = default;

  FrenetFramePoint(const double s, const double l, const double dl,
                   const double ddl);

  void set_dl(const double dl);
  void set_ddl(const double ddl);

  void set_param(const double s, const double l, const double dl,
                 const double ddl);
  void set_heading_diff(const double heading_diff);

  double dl() const;
  double ddl() const;
  double heading_diff() const;

 private:
  double dl_ = 0.0;
  double ddl_ = 0.0;
  double heading_diff_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
