#pragma once

#include <algorithm>

#include "path.h"
#include "path_point.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class DiscretizedPath : public Path {
 public:
  DiscretizedPath() = default;

  DiscretizedPath(const std::vector<PathPoint>& path_points);

  virtual ~DiscretizedPath() override = default;

  virtual bool evaluate(const double param, PathPoint& pt) const override;

  virtual double param_length() const override;

  virtual bool start_point(PathPoint& pt) const override;

  virtual bool end_point(PathPoint& pt) const override;

  bool evaluate_linear_approximation(const double param, PathPoint& pt) const;

  void set_path_points(const std::vector<PathPoint>& path_points);

  int query_closest_point(const double param) const;
  bool query_closest_point(const Vec2d& pos, PathPoint& pt) const;
  bool QueryClosestPointWithIndex(const Vec2d& pos, PathPoint& pt,
                                      int& index) const;

  std::vector<PathPoint>* mutable_path_points();

  const std::vector<PathPoint>& path_points() const;

  std::size_t num_of_points() const;

  bool path_point_at(const std::size_t index, PathPoint& pt) const;

  void clear();

 private:
  std::vector<PathPoint>::const_iterator query_lower_bound(
      const double param) const;

  std::vector<PathPoint> path_points_;
};

}  // namespace planning
}  // namespace neodrive
