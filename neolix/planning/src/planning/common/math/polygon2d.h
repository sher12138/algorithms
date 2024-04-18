#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "aabox2d.h"
#include "box2d.h"
#include "common/math/math_utils.h"
#include "segment2d.h"
#include "common/math/double.h"
#include "vec2d.h"

namespace neodrive {
namespace planning {
// TODO(wwl) : using Google naming rule
class Polygon2d {
 public:
  Polygon2d() = default;
  explicit Polygon2d(const Box2d &box);
  // TODO(wwl) : reference is preferred to be passed into
  explicit Polygon2d(std::vector<Vec2d> &&points);
  explicit Polygon2d(const std::vector<Vec2d> &points);
  void Init(const Box2d &box);
  void Init(std::vector<Vec2d> &&points);
  void Init(const std::vector<Vec2d> &points);

  const std::vector<Vec2d> &points() const { return points_; }
  const std::vector<Segment2d> &segments() const { return segments_; }
  int num_points() const { return num_points_; }
  bool is_convex() const { return is_convex_; }
  double area() const { return area_; }

  double min_x() const { return min_x_; }
  double max_x() const { return max_x_; }
  double min_y() const { return min_y_; }
  double max_y() const { return max_y_; }

  double distance_to_boundary(const Vec2d &point) const;
  double distance_to(const Vec2d &point) const;
  double distance_to(const Segment2d &segment) const;
  double distance_to(const Box2d &box) const;
  double distance_to(const Polygon2d &polygon) const;
  double distance_to_2(const Polygon2d &polygon) const;
  double distance_to_2(const Segment2d &segment) const;

  double distance_sqr_to(const Box2d &box) const;
  double distance_sqr_to(const Polygon2d &polygon) const;
  double distance_sqr_to(const Segment2d &segment) const;
  double distance_sqr_to(const Vec2d &point) const;

  bool is_point_in(const Vec2d &point) const;
  bool is_point_on_boundary(const Vec2d &point) const;

  bool is_contain(const Segment2d &segment) const;
  bool is_contain(const Polygon2d &polygon) const;

  static bool compute_convex_hull(const std::vector<Vec2d> &points,
                                  Polygon2d *const polygon);

  bool has_overlap(const Segment2d &segment) const;
  bool get_overlap(const Segment2d &segment, Vec2d *const first,
                   Vec2d *const last) const;
  std::vector<Segment2d> get_all_overlaps(const Segment2d &segment) const;

  bool has_overlap(const Segment2d &line_segment,
                   bool judge_point_in_flag) const;

  bool get_overlap(const Segment2d &line_segment,
                   bool judge_point_in_flag = true) const;

  bool has_overlap(const Polygon2d &polygon) const;

  // ploygon overlap with purning
  bool has_overlap_2(const Polygon2d &polygon) const;

  // Only compute overlaps between two convex polygons.
  bool compute_overlap(const Polygon2d &other_polygon,
                       Polygon2d *const overlap_polygon) const;

  // Only compute intersection over union ratio between two convex polygons.
  /**
   * @brief Compute intersection over union ratio of this polygon and the other
   * polygon. Note: this function only works for computing overlap
   * between two convex polygons.
   * @param other_polygon The target polygon. To compute its overlap with
   *        this polygon.
   * @return A value between 0.0 and 1.0, meaning no intersection to fully
   * overlaping
   */
  double compute_IoU(const Polygon2d &other_polygon) const;

  AABox2d aa_bounding_box() const;
  Box2d bounding_box_with_heading(const double heading) const;
  Box2d min_area_bounding_box() const;
  void extreme_points(const double heading, Vec2d *const first,
                      Vec2d *const last) const;

  Polygon2d expand_by_distance(const double distance) const;

  std::string debug_string() const;

 protected:
  void build_from_points();

 protected:
  int next(int at) const;
  int prev(int at) const;

  // Remove point p if and only if outer_prod(segment.start, segment.end, p) <
  // 0.
  static bool clip_convex_hull(const Segment2d &segment,
                               std::vector<Vec2d> *const points);

  std::vector<Vec2d> points_;
  int num_points_ = 0;
  std::vector<Segment2d> segments_;
  bool is_convex_ = false;
  double area_ = 0.0;
  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
