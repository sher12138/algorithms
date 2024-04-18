#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "common/math/math_utils.h"
#include "vec2d.h"

namespace neodrive {
namespace planning {

class Segment2d {
 public:
  Segment2d();
  Segment2d(const Vec2d& start, const Vec2d& end);

  const Vec2d& start() const { return start_; }
  const Vec2d& end() const { return end_; }
  const Vec2d& unit_direction() const { return unit_direction_; }
  Vec2d center() const { return (start_ + end_) / 2.0; }
  double heading() const { return heading_; }
  double cos_heading() const { return unit_direction_.x(); }
  double sin_heading() const { return unit_direction_.y(); }

  double length() const;
  double length_sqr() const;

  double distance_to(const Vec2d& point) const;
  double distance_to(const Vec2d& point, Vec2d* const nearest_pt) const;
  double distance_sqr_to(const Vec2d& point) const;
  double distance_sqr_to(const Vec2d& point, Vec2d* const nearest_pt) const;

  bool is_point_in(const Vec2d& point) const;

  bool has_intersect(const Segment2d& other_segment) const;
  bool get_intersect(const Segment2d& other_segment, Vec2d* const point) const;

  double project_onto_unit(const Vec2d& point) const;
  double product_onto_unit(const Vec2d& point) const;
  double get_perpendicular_foot(const Vec2d& point,
                                Vec2d* const foot_point) const;

  std::string debug_string() const;

 protected:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
