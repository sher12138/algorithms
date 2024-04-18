#include "vec2d.h"

namespace neodrive {
namespace planning {

Vec2d Vec2d::create_unit_vec(const double angle) {
  return Vec2d(cos(angle), sin(angle));
}

double Vec2d::length() const {
  // return hypot(x_, y_);
  // BING: use faster function
  return sqrt(x_ * x_ + y_ * y_);
}

double Vec2d::length_sqr() const { return x_ * x_ + y_ * y_; }

double Vec2d::distance_to(const Vec2d& other) const {
  // return hypot(x_ - other.x_, y_ - other.y_);
  // BING: use faster function
  return sqrt((x_ - other.x_) * (x_ - other.x_) +
              (y_ - other.y_) * (y_ - other.y_));
}

double Vec2d::distance_sqr_to(const Vec2d& other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

void Vec2d::normalize() {
  const double l = length();
  if (l > kMathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

Vec2d& Vec2d::operator=(const Vec2d& rhs) {
  this->x_ = rhs.x();
  this->y_ = rhs.y();
  return *this;
}

Vec2d Vec2d::operator+(const Vec2d& other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-(const Vec2d& other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const {
  if (std::abs(ratio) <= kMathEpsilon) {
    LOG_ERROR("ratio [{:.4f}] <= kMathEpsilon", std::abs(ratio));
    Vec2d p;
    return p;
  }
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d& Vec2d::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d& Vec2d::operator/=(const double ratio) {
  if (std::abs(ratio) <= kMathEpsilon) {
    LOG_ERROR("ratio [{:.4f}] <= kMathEpsilon", std::abs(ratio));
    return *this;
  }
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d& other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

double Vec2d::cross_prod(const Vec2d& other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::inner_prod(const Vec2d& other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2d Vec2d::rotate(const double angle) const {
  return Vec2d(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::self_rotate(const double angle) {
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d operator*(const double ratio, const Vec2d& vec) { return vec * ratio; }

std::string Vec2d::debug_string() const {
  std::ostringstream sout;
  sout << "vec2d ( x = " << x_ << "  y = " << y_ << " )";
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace neodrive
