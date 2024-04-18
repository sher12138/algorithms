#include "aabox3d.h"

#include "neolix_log.h"
#include "common/math/math_parameters.h"

namespace neodrive {
namespace planning {

AABox3d::AABox3d(const Vec3d& center, const double length, const double width,
                 const double height)
    : center_{center}, length_{length}, width_{width}, height_{height},
      half_length_{length / 2.0}, half_width_{width / 2.0},
      half_height_{height / 2} {
  if (length_ < -kMathEpsilon || width_ < -kMathEpsilon ||
      height_ < -kMathEpsilon) {
    LOG_ERROR("length_: {:.4f}, width_: {:.4f}, height_: {:.4f} error",
              length_, width_, height_);
  }
}

AABox3d::AABox3d(const Vec3d& one, const Vec3d& other)
    : AABox3d{(one + other) / 2, std::abs(one.x() - other.x()),
              std::abs(one.y() - other.y()), std::abs(one.z() - other.z())} {}

AABox3d::AABox3d(const std::vector<Vec3d>& points) {
  assert(points.size());

  auto& p0 = points[0];
  auto [minx, miny, minz, maxx, maxy, maxz] = std::array{
    p0.x(), p0.y(), p0.z(), p0.x(), p0.y(), p0.z()};
  for (auto& p : points) {
    std::tie(minx, miny, minz) = std::tuple{
      std::min(minx, p.x()), std::min(miny, p.y()), std::min(minz, p.z())};
    std::tie(maxx, maxy, maxz) = std::tuple{
      std::max(maxx, p.x()), std::max(maxy, p.y()), std::max(maxz, p.z())};
  }

  center_ = {(minx + maxx) / 2, (miny + maxy) / 2, (minz + maxz) / 2};
  std::tie(length_, width_, height_) = std::tuple{
      maxx - minx, maxy - miny, maxz - minz};
  std::tie(half_length_, half_width_, half_height_) = std::tuple{
      length_ / 2, width_ / 2, height_ / 2};
}

const Vec3d& AABox3d::center() const { return center_; }

double AABox3d::length() const { return length_; }

double AABox3d::width() const { return width_; }

double AABox3d::height() const { return height_; }

double AABox3d::half_length() const { return half_length_; }

double AABox3d::half_width() const { return half_width_; }

double AABox3d::half_height() const { return half_height_; }

double AABox3d::volume() const { return length_ * width_ * height_; }

double AABox3d::min_x() const { return center_.x() - half_length_; }

double AABox3d::max_x() const { return center_.x() + half_length_; }

double AABox3d::min_y() const { return center_.y() - half_width_; }

double AABox3d::max_y() const { return center_.y() + half_width_; }

double AABox3d::min_z() const { return center_.z() - half_height_; }

double AABox3d::max_z() const { return center_.z() + half_height_; }

void AABox3d::get_all_corners(std::vector<Vec3d>* const corners) const {
  auto& c = center_;
  *corners = {
    {c.x() + half_length_, c.y() - half_width_, c.z() - half_height_},
    {c.x() + half_length_, c.y() + half_width_, c.z() - half_height_},
    {c.x() - half_length_, c.y() + half_width_, c.z() - half_height_},
    {c.x() - half_length_, c.y() - half_width_, c.z() - half_height_},
    {c.x() + half_length_, c.y() - half_width_, c.z() + half_height_},
    {c.x() + half_length_, c.y() + half_width_, c.z() + half_height_},
    {c.x() - half_length_, c.y() + half_width_, c.z() + half_height_},
    {c.x() - half_length_, c.y() - half_width_, c.z() + half_height_},
  };
}

bool AABox3d::is_point_in(const Vec3d& point) const {
  auto d = point - center_;
  return std::abs(d.x()) - half_length_ < kMathEpsilon &&
         std::abs(d.y()) - half_width_ < kMathEpsilon &&
         std::abs(d.z()) - half_height_ < kMathEpsilon;
}

double AABox3d::distance_to(const Vec3d& point) const {
  Vec3d vec{
    std::abs(point.x() - center_.x()) - half_length_,
    std::abs(point.y() - center_.y()) - half_width_,
    std::abs(point.z() - center_.z()) - half_height_,
  };
  Vec3d len{std::max(0., vec.x()), std::max(0., vec.y()),
            std::max(0., vec.z())};
  return len.length();
}

double AABox3d::distance_to(const AABox3d& box) const {
  Vec3d vec{
    std::abs(box.center().x() - center_.x()) - box.half_length() - half_length_,
    std::abs(box.center().y() - center_.y()) - box.half_width() - half_width_,
    std::abs(box.center().z() - center_.z()) - box.half_height() - half_height_
  };
  Vec3d len{std::max(0., vec.x()), std::max(0., vec.y()),
            std::max(0., vec.z())};
  return len.length();
}

bool AABox3d::has_overlap(const AABox3d& box) const {
  return std::abs(box.center().x() - center_.x()) <
             box.half_length() + half_length_ &&
         std::abs(box.center().y() - center_.y()) <
             box.half_width() + half_width_ &&
         std::abs(box.center().z() - center_.z()) <
             box.half_height() + half_height_;
}

void AABox3d::shift(const Vec3d& shift_vec) { center_ += shift_vec; }

void AABox3d::merge_from(const AABox3d& other) {
  auto [minx, miny, minz, maxx, maxy, maxz] = std::array{
    std::min(min_x(), other.min_x()), std::min(min_y(), other.min_y()),
    std::min(min_z(), other.min_z()), std::max(max_x(), other.max_x()),
    std::max(max_y(), other.max_y()), std::max(max_z(), other.max_z())};

  center_ = Vec3d{minx + maxx, miny + maxy, minz + maxz} / 2;
  std::tie(length_, width_, height_) = std::tuple{
    maxx - minx, maxy - miny, maxz - minz};
  std::tie(half_length_, half_width_, half_height_) = std::tuple{
    length_ / 2, width_ / 2, height_ / 2};
}

void AABox3d::merge_from(const Vec3d& other) {
  auto [minx, miny, minz, maxx, maxy, maxz] = std::array{
    std::min(min_x(), other.x()), std::min(min_y(), other.y()),
    std::min(min_z(), other.z()), std::max(max_x(), other.x()),
    std::max(max_y(), other.y()), std::max(max_z(), other.z())};

  center_ = Vec3d{minx + maxx, miny + maxy, minz + maxz} / 2;
  std::tie(length_, width_, height_) = std::tuple{
    maxx - minx, maxy - miny, maxz - minz};
  std::tie(half_length_, half_width_, half_height_) = std::tuple{
    length_ / 2, width_ / 2, height_ / 2};
}

}  // namespace planning
}  // namespace neodrive
