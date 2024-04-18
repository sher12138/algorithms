#pragma once

#include <algorithm>
#include <limits>

#include "src/planning/common/path/sl_point.h"

namespace neodrive {
namespace planning {

//////////////////////////////////
////     end_s
////       ^
////       |
//// S path direction
////       |
////   start_s
////
////  end_l  <-----L direction---- start_l
////////////////////////////////////
class Boundary {
 public:
  Boundary() = default;
  ~Boundary() = default;

  Boundary(const double s_s, const double e_s, const double s_l,
           const double e_l);

  void reset();

  double start_s() const;
  double end_s() const;
  double start_l() const;
  double end_l() const;

  void set_start_s(const double start_s);
  void set_end_s(const double end_s);
  void set_start_l(const double start_l);
  void set_end_l(const double end_l);

  bool has_lateral_overlap(const Boundary& boundary) const;
  bool has_horizontal_overlap(const Boundary& boundary) const;
  bool has_overlap(const Boundary& boundary) const;

  bool is_point_in(const SLPoint& pt) const;
  bool is_s_in(const double s) const;
  bool is_l_in(const double l) const;

  double lateral_distance(const Boundary& boundary) const;

  void expand(const double expand_dist);
  void left_expand(const double expand_dist);
  void right_expand(const double expand_dist);
  void front_expand(const double expand_dist);
  void back_expand(const double expand_dist);

  void merge(const Boundary& boundary);
  double distance_to(const Boundary& other) const;
  double distance_to(const Vec2d& point) const;

 private:
  double start_s_ = std::numeric_limits<double>::max();
  double end_s_ = std::numeric_limits<double>::lowest();
  double start_l_ = std::numeric_limits<double>::max();
  double end_l_ = std::numeric_limits<double>::lowest();
};

}  // namespace planning
}  // namespace neodrive
