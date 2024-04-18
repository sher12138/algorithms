#pragma once

#include <sstream>

#include <json/json.h>
#include "vec2d.h"

namespace neodrive {
namespace planning_rl {

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

}  // namespace planning_rl
}  // namespace neodrive
