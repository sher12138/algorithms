#pragma once

#include <algorithm>
#include <vector>

#include "frenet_frame_point.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class FrenetFramePath {
 public:
  FrenetFramePath() = default;

  FrenetFramePath(const std::vector<FrenetFramePoint>& sl_points);

  ~FrenetFramePath() = default;

  void set_frenet_points(const std::vector<FrenetFramePoint>& points);

  std::vector<FrenetFramePoint>* mutable_points();

  const std::vector<FrenetFramePoint>& points() const;

  std::size_t num_of_points() const;

  bool point_at(const std::size_t index, FrenetFramePoint& pt) const;

  bool interpolate(const double s, FrenetFramePoint& pt) const;

  void clear();

 private:
  std::vector<FrenetFramePoint> points_;
};

}  // namespace planning
}  // namespace neodrive
