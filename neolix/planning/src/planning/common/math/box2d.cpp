#include "box2d.h"

#include "polygon2d.h"

namespace neodrive {
namespace planning {

Box2d::Box2d(const Vec2d& center, const double heading, const double length,
             const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon) {
    LOG_ERROR("length_: {:.4f}, width_: {:.4f} error", length_, width_);
  }
}

Box2d::Box2d(const Segment2d& axis, const double width)
    : center_(axis.center()),
      length_(axis.length()),
      width_(width),
      half_length_(axis.length() / 2.0),
      half_width_(width / 2.0),
      heading_(axis.heading()),
      cos_heading_(axis.cos_heading()),
      sin_heading_(axis.sin_heading()) {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon) {
    LOG_ERROR("length_: {:.4f}, width_: {:.4f} error", length_, width_);
  }
}

Box2d::Box2d(const AABox2d& aabox)
    : center_(aabox.center()),
      length_(aabox.length()),
      width_(aabox.width()),
      half_length_(aabox.half_length()),
      half_width_(aabox.half_width()),
      heading_(0.0),
      cos_heading_(1.0),
      sin_heading_(0.0) {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon) {
    LOG_ERROR("length_: {:.4f}, width_: {:.4f} error", length_, width_);
  }
}

Box2d Box2d::create_aa_box(const Vec2d& one_corner,
                           const Vec2d& opponent_corner) {
  const double x1 = std::min(one_corner.x(), opponent_corner.x());
  const double x2 = std::max(one_corner.x(), opponent_corner.x());
  const double y1 = std::min(one_corner.y(), opponent_corner.y());
  const double y2 = std::max(one_corner.y(), opponent_corner.y());
  return Box2d({(x1 + x2) / 2.0, (y1 + y2) / 2.0}, 0.0, x2 - x1, y2 - y1);
}

void Box2d::get_all_corners(std::vector<Vec2d>* const corners) const {
  if (corners == nullptr) {
    return;
  }
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners->clear();
  corners->reserve(4);
  // do not change the sequence of four corner points:
  // right front pt.
  corners->emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  // left front pt.
  corners->emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  // left back pt.
  corners->emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  // right back pt.
  corners->emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);
}

bool Box2d::is_point_in(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (dx <= half_length_ + kMathEpsilon) &&
         (dy <= half_width_ + kMathEpsilon);
}

bool Box2d::is_point_on_boundary(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

double Box2d::distance_to(const Vec2d& point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool Box2d::has_overlap(const Segment2d& segment) const {
  if (segment.length() <= kMathEpsilon) {
    return is_point_in(segment.start());
  }
  return distance_to(segment) <= kMathEpsilon;
}

double Box2d::distance_to(const Segment2d& segment) const {
  if (segment.length() <= kMathEpsilon) {
    return distance_to(segment.start());
  }
  const double ref_x1 = segment.start().x() - center_.x();
  const double ref_y1 = segment.start().y() - center_.y();
  double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
  double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
  double box_x = half_length_;
  double box_y = half_width_;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const double ref_x2 = segment.end().x() - center_.x();
  const double ref_y2 = segment.end().y() - center_.y();
  double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
  double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return pt_seg_distance(box_x, box_y, x1, y1, x2, y2, segment.length());
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : pt_seg_distance(box_x, box_y, x1, y1, x2, y2,
                                           segment.length());
      case 2:
        return (x1 > x2) ? pt_seg_distance(box_x, -box_y, x1, y1, x2, y2,
                                           segment.length())
                         : pt_seg_distance(box_x, box_y, x1, y1, x2, y2,
                                           segment.length());
      case -1:
        return outer_prod(x1, y1, x2, y2, box_x, -box_y) >= 0.0
                   ? 0.0
                   : pt_seg_distance(box_x, -box_y, x1, y1, x2, y2,
                                     segment.length());
      case -4:
        return outer_prod(x1, y1, x2, y2, box_x, -box_y) <= 0.0
                   ? pt_seg_distance(box_x, -box_y, x1, y1, x2, y2,
                                     segment.length())
                   : (outer_prod(x1, y1, x2, y2, -box_x, box_y) <= 0.0
                          ? 0.0
                          : pt_seg_distance(-box_x, box_y, x1, y1, x2, y2,
                                            segment.length()));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : pt_seg_distance(box_x, box_y, x1, y1, x2, y2,
                                           segment.length());
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return outer_prod(x1, y1, x2, y2, box_x, box_y) <= 0.0
                   ? 0.0
                   : pt_seg_distance(box_x, box_y, x1, y1, x2, y2,
                                     segment.length());
      case -3:
        return 0.0;
    }
  }
  LOG_ERROR("unimplemented state: gx1: {}, gy1: {}, gx2: {}, gy2: {}", gx1, gy1,
            gx2, gy2);
  return 0.0;
}

double Box2d::distance_to(const Box2d& box) const {
  return Polygon2d(box).distance_to(*this);
}

double Box2d::distance_sqr_to(const Box2d& box) const {
  return Polygon2d(box).distance_sqr_to(*this);
}

// SAT: Separating Axis Theorem
bool Box2d::has_overlap(const Box2d& box) const {
  // use aabox to reduce complexity: it cannot reduce
  //  AABox2d self_aabox = get_aa_box();
  //  AABox2d traget_aabox = box.get_aa_box();
  //  if (!self_aabox.has_overlap(traget_aabox)) {
  //    return false;
  //  }
  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

AABox2d Box2d::get_aa_box() const {
  const double dx1 = std::abs(cos_heading_ * half_length_);
  const double dy1 = std::abs(sin_heading_ * half_length_);
  const double dx2 = std::abs(sin_heading_ * half_width_);
  const double dy2 = std::abs(cos_heading_ * half_width_);
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

void Box2d::rotate_from_center(double rotate_angle) {
  heading_ = normalize_angle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
}

void Box2d::shift(const Vec2d& shift_vec) { center_ += shift_vec; }

std::string Box2d::debug_string() const {
  std::ostringstream sout;
  sout << "box2d ( center = " << center_.debug_string()
       << "  heading = " << heading_ << "  length = " << length_
       << "  width = " << width_ << " )";
  sout.flush();
  return sout.str();
}

double Box2d::outer_prod(double x0, double y0, double x1, double y1, double x2,
                         double y2) const {
  return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
}

double Box2d::pt_seg_distance(double query_x, double query_y, double start_x,
                              double start_y, double end_x, double end_y,
                              double length) const {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double proj = x0 * dx + y0 * dy;
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

}  // namespace planning
}  // namespace neodrive
