#include "vec3d.h"

#include <cmath>
#include <tuple>

namespace neodrive {
namespace planning {

double Vec3d::length() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_); }

Vec3d& Vec3d::operator=(const Vec3d& rhs) {
  this->x_ = rhs.x();
  this->y_ = rhs.y();
  this->z_ = rhs.z();
  return *this;
}

Vec3d Vec3d::operator+(const Vec3d& other) const {
  return {x_ + other.x(), y_ + other.y(), z_ + other.z()};
}

Vec3d Vec3d::operator-(const Vec3d& other) const {
  return {x_ - other.x(), y_ - other.y(), z_ - other.z()};
}

Vec3d Vec3d::operator*(const double ratio) const {
  return {x_ * ratio, y_ * ratio, z_ * ratio};
}

Vec3d Vec3d::operator/(const double ratio) const {
  return {x_ / ratio, y_ / ratio, z_ / ratio};
}

Vec3d& Vec3d::operator+=(const Vec3d& o) {
  std::tie(x_, y_, z_) = std::tuple{x_ + o.x(), y_ + o.y(), z_ + o.z()};
  return *this;
}

Vec3d& Vec3d::operator-=(const Vec3d& o) {
  std::tie(x_, y_, z_) = std::tuple{x_ - o.x(), y_ - o.y(), z_ - o.z()};
  return *this;
}

Vec3d& Vec3d::operator*=(const double r) {
  std::tie(x_, y_, z_) = std::tuple{x_ * r, y_ * r, z_ * r};
  return *this;
}

Vec3d& Vec3d::operator/=(const double r) {
  std::tie(x_, y_, z_) = std::tuple{x_ / r, y_ / r, z_ / r};
  return *this;
}

}  // namespace planning
}  // namespace neodrive
