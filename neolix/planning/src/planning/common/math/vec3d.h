/// Define 3d vector
#pragma once

#include "common/macros.h"

namespace neodrive {
namespace planning {

class Vec3d {
 public:
  /// Construct with x, y, z
  constexpr Vec3d(const double x, const double y, const double z) noexcept
      : x_(x), y_(y), z_(z) {}
  constexpr Vec3d() noexcept : x_(0.), y_(0.), z_(0.) {}
  Vec3d(const Vec3d& rhs) : x_(rhs.x()), y_(rhs.y()), z_(rhs.z()) {}

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, x);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, y);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, z);

  /// Get length of vector
  double length() const;

  Vec3d& operator=(const Vec3d& rhs);
  /// Operator
  Vec3d operator+(const Vec3d& other) const;

  /// Operator
  Vec3d operator-(const Vec3d& other) const;

  /// Operator
  Vec3d operator*(const double ratio) const;

  /// Operator
  Vec3d operator/(const double ratio) const;

  /// Operator
  Vec3d& operator+=(const Vec3d& other);

  /// Operator
  Vec3d& operator-=(const Vec3d& other);

  /// Operator
  Vec3d& operator*=(const double ratio);

  /// Operator
  Vec3d& operator/=(const double ratio);

 private:
  double x_{0};
  double y_{0};
  double z_{0};
};

}  // namespace planning
}  // namespace neodrive
