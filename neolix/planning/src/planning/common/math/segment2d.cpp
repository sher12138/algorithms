#include "segment2d.h"

namespace neodrive {
namespace planning {

namespace {

bool is_within(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

}  // namespace

Segment2d::Segment2d() { unit_direction_ = Vec2d(1, 0); }

Segment2d::Segment2d(const Vec2d& start, const Vec2d& end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.angle();
}

double Segment2d::length() const { return length_; }

double Segment2d::length_sqr() const { return length_ * length_; }

double Segment2d::distance_to(const Vec2d& point) const {
  if (length_ <= kMathEpsilon) {
    return point.distance_to(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    // return hypot(x0, y0);
    // BING: use faster function
    return sqrt(x0 * x0 + y0 * y0);
  }
  if (proj >= length_) {
    return point.distance_to(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::distance_to(const Vec2d& point,
                              Vec2d* const nearest_pt) const {
  if (nearest_pt == nullptr) {
    return 0.0;
  }
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.distance_to(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    // return hypot(x0, y0);
    // BING: use faster function
    return sqrt(x0 * x0 + y0 * y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.distance_to(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::distance_sqr_to(const Vec2d& point) const {
  if (length_ <= kMathEpsilon) {
    return point.distance_sqr_to(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return sqr(x0) + sqr(y0);
  }
  if (proj >= length_) {
    return point.distance_sqr_to(end_);
  }
  return sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::distance_sqr_to(const Vec2d& point,
                                  Vec2d* const nearest_pt) const {
  if (nearest_pt == nullptr) {
    return 0.0;
  }
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.distance_sqr_to(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return sqr(x0) + sqr(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.distance_sqr_to(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool Segment2d::is_point_in(const Vec2d& point) const {
  if (length_ <= kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon;
  }
  const double prod = cross_prod(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon) {
    return false;
  }
  return is_within(point.x(), start_.x(), end_.x()) &&
         is_within(point.y(), start_.y(), end_.y());
}

double Segment2d::project_onto_unit(const Vec2d& point) const {
  return unit_direction_.inner_prod(point - start_);
}

double Segment2d::product_onto_unit(const Vec2d& point) const {
  return unit_direction_.cross_prod(point - start_);
}

bool Segment2d::has_intersect(const Segment2d& other_segment) const {
  Vec2d point;
  return get_intersect(other_segment, &point);
}

bool Segment2d::get_intersect(const Segment2d& other_segment,
                              Vec2d* const point) const {
  if (point == nullptr) {
    return false;  // invalid input, suppose they are intersected
  }
  if (is_point_in(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (is_point_in(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.is_point_in(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.is_point_in(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
    return false;
  }
  const double cc1 = cross_prod(start_, end_, other_segment.start());
  const double cc2 = cross_prod(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon) {
    return false;
  }
  const double cc3 =
      cross_prod(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      cross_prod(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double Segment2d::get_perpendicular_foot(const Vec2d& point,
                                         Vec2d* const foot_point) const {
  if (foot_point == nullptr) {
    return 0.0;
  }
  if (length_ <= kMathEpsilon) {
    *foot_point = start_;
    return point.distance_to(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

std::string Segment2d::debug_string() const {
  std::ostringstream sout;
  sout << "segment2d (" << start_.debug_string() << ", " << end_.debug_string()
       << ")";
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace neodrive
