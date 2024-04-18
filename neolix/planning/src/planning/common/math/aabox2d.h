#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "math_parameters.h"
#include "neolix_log.h"
#include "vec2d.h"

namespace neodrive {
namespace planning {
// TODO(wwl) : using Google naming rule
class AABox2d {
 public:
  AABox2d() = default;
  AABox2d(const Vec2d& center, const double length, const double width);
  AABox2d(const Vec2d& one_corner, const Vec2d& opponent_corner);
  AABox2d(const std::vector<Vec2d>& points);
  void set_id(const int id) { id_ = id; }
  void set_obs_type(const int obs_type) { obs_type_ = obs_type; }

  const Vec2d& center() const { return center_; }
  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  double length() const { return length_; }
  double width() const { return width_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double area() const { return length_ * width_; }
  double min_x() const { return center_.x() - half_length_; }
  double max_x() const { return center_.x() + half_length_; }
  double min_y() const { return center_.y() - half_width_; }
  double max_y() const { return center_.y() + half_width_; }
  int id() const { return id_; }
  int obs_type() const { return obs_type_; }

  // Get all corners in counter clockwise order.
  void get_all_corners(std::vector<Vec2d>* const corners) const;

  bool is_point_in(const Vec2d& point) const;
  bool is_point_on_boundary(const Vec2d& point) const;

  double distance_to(const Vec2d& point) const;
  double distance_to(const AABox2d& box) const;

  bool has_overlap(const AABox2d& box) const;

  // Shift the center of AABox by the input vector.
  void shift(const Vec2d& shift_vec);

  void merge_from(const AABox2d& other_box);
  void merge_from(const Vec2d& other_point);

  std::string debug_string() const;
  std::string points_debug_string() const;

 protected:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  // spacial for path region search
  int id_{0};
  int obs_type_{-1};  // corresponding Obstacle::ObstacleType
};

}  // namespace planning
}  // namespace neodrive
