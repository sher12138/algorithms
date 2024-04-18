#pragma once

#include <iomanip>
#include <sstream>

#include "common/math/vec2d.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class STPoint : public Vec2d {
 public:
  STPoint() = default;
  ~STPoint() = default;

  STPoint(const double s, const double t);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
};

}  // namespace planning
}  // namespace neodrive
