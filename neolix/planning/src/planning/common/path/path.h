#pragma once

#include "path_point.h"

namespace neodrive {
namespace planning {

class Path {
 public:
  Path() = default;

  virtual ~Path() = default;

  virtual bool evaluate(const double param, PathPoint& pt) const = 0;

  virtual double param_length() const = 0;

  virtual bool start_point(PathPoint& pt) const = 0;

  virtual bool end_point(PathPoint& pt) const = 0;
};

}  // namespace planning
}  // namespace neodrive
