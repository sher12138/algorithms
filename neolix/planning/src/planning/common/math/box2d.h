#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "aabox2d.h"
#include "segment2d.h"
#include "vec2d.h"

namespace neodrive {
namespace planning {

class Box2d {
 public:
  Box2d(const Vec2d& center, const double heading, const double length,
        const double width);
  Box2d(const Segment2d& axis, const double width);
  explicit Box2d(const AABox2d& aabox);
  static Box2d create_aa_box(const Vec2d& one_corner,
                             const Vec2d& opponent_corner);

  const Vec2d& center() const { return center_; }
  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  double length() const { return length_; }
  double width() const { return width_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double heading() const { return heading_; }
  double cos_heading() const { return cos_heading_; }
  double sin_heading() const { return sin_heading_; }
  double area() const { return length_ * width_; }
  double diagonal() const { return hypot(length_, width_); }

  void get_all_corners(std::vector<Vec2d>* const corners) const;

  bool is_point_in(const Vec2d& point) const;
  bool is_point_on_boundary(const Vec2d& point) const;

  double distance_to(const Vec2d& point) const;
  double distance_to(const Segment2d& segment) const;
  double distance_to(const Box2d& box) const;
  double distance_sqr_to(const Box2d& box) const;

  bool has_overlap(const Segment2d& segment) const;
  bool has_overlap(const Box2d& box) const;

  AABox2d get_aa_box() const;

  void rotate_from_center(double rotate_angle);
  void shift(const Vec2d& shift_vec);

  std::string debug_string() const;

 private:
  double outer_prod(double x0, double y0, double x1, double y1, double x2,
                    double y2) const;
  double pt_seg_distance(double query_x, double query_y, double start_x,
                         double start_y, double end_x, double end_y,
                         double length) const;

 protected:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
