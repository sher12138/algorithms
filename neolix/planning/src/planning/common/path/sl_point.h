#pragma once

#include <sstream>

#include "common/math/vec2d.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class SLPoint : public Vec2d {
 public:
  SLPoint() = default;
  SLPoint(const double s, const double l);
  virtual ~SLPoint() = default;

  double s() const;
  double l() const;
  void set_s(const double s);
  void set_l(const double l);

  static SLPoint interpolate(const SLPoint& start, const SLPoint& end,
                             const double weight);

  virtual Json::Value to_json() const;
};

}  // namespace planning
}  // namespace neodrive
