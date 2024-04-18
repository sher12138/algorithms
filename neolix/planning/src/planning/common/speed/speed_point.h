#pragma once

#include "st_point.h"

namespace neodrive {
namespace planning {

class SpeedPoint : public STPoint {
 public:
  SpeedPoint() = default;
  ~SpeedPoint() = default;

  SpeedPoint(const double s, const double t, const double v, const double a,
             const double j);

  SpeedPoint(const STPoint& st_point, const double v, const double a,
             const double j);

  void set_v(const double v);
  void set_a(const double a);
  void set_j(const double j);

  double v() const;
  double a() const;
  double j() const;

  static SpeedPoint interpolate(const SpeedPoint& left, const SpeedPoint& right,
                                const double weight);

 private:
  double v_ = 0.0;
  double a_ = 0.0;
  double j_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
