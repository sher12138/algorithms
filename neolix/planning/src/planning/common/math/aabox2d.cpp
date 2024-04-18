#include "aabox2d.h"

namespace neodrive {
namespace planning {

AABox2d::AABox2d(const Vec2d& center, const double length, const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0) {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon) {
    LOG_ERROR("length_: {:.4f}, width_: {:.4f} error", length_, width_);
  }
}

AABox2d::AABox2d(const Vec2d& one_corner, const Vec2d& opponent_corner)
    : AABox2d((one_corner + opponent_corner) / 2.0,
              std::abs(one_corner.x() - opponent_corner.x()),
              std::abs(one_corner.y() - opponent_corner.y())) {}

AABox2d::AABox2d(const std::vector<Vec2d>& points) {
  if (points.empty()) {
    LOG_ERROR("points.empty()");
    return;
  }
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto& point : points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  center_ = {(min_x + max_x) / 2.0, (min_y + max_y) / 2.0};
  length_ = max_x - min_x;
  width_ = max_y - min_y;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::get_all_corners(std::vector<Vec2d>* const corners) const {
  if (corners == nullptr) {
    LOG_ERROR("input is nullptr");
    return;
  }
  corners->clear();
  corners->emplace_back(
      Vec2d(center_.x() + half_length_, center_.y() - half_width_));
  corners->emplace_back(
      Vec2d(center_.x() + half_length_, center_.y() + half_width_));
  corners->emplace_back(
      Vec2d(center_.x() - half_length_, center_.y() + half_width_));
  corners->emplace_back(
      Vec2d(center_.x() - half_length_, center_.y() - half_width_));
}

bool AABox2d::is_point_in(const Vec2d& point) const {
  return std::abs(point.x() - center_.x()) <= half_length_ + kMathEpsilon &&
         std::abs(point.y() - center_.y()) <= half_width_ + kMathEpsilon;
}

bool AABox2d::is_point_on_boundary(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

double AABox2d::distance_to(const Vec2d& point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

double AABox2d::distance_to(const AABox2d& box) const {
  const double dx =
      std::abs(box.center_x() - center_.x()) - box.half_length() - half_length_;
  const double dy =
      std::abs(box.center_y() - center_.y()) - box.half_width() - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool AABox2d::has_overlap(const AABox2d& box) const {
  return (std::abs(box.center_x() - center_.x()) <=
          box.half_length() + half_length_ + kMathEpsilon) &&
         (std::abs(box.center_y() - center_.y()) <=
          box.half_width() + half_width_ + kMathEpsilon);
}

void AABox2d::shift(const Vec2d& shift_vec) { center_ += shift_vec; }

void AABox2d::merge_from(const AABox2d& other_box) {
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::merge_from(const Vec2d& other_point) {
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

std::string AABox2d::debug_string() const {
  std::ostringstream sout;
  sout << "aabox2d ( center = " << center_.debug_string()
       << "  length = " << length_ << "  width = " << width_ << " )";
  sout.flush();
  return sout.str();
}

std::string AABox2d::points_debug_string() const {
  std::ostringstream sout;
  sout << "points = (" << min_x() << ", " << min_y() << ") (" << min_x() << ", "
       << max_y() << ") (" << max_x() << ", " << max_y() << ") (" << max_x()
       << ", " << min_y() << ")";
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace neodrive
