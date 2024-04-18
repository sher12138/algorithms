#pragma once

#include <cmath>
#include <sstream>
#include <string>

#include "common/macros.h"
#include "math_parameters.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class Vec2d {
 public:
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}
  Vec2d(const Vec2d& rhs) : x_(rhs.x()), y_(rhs.y()) {}
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}
  static Vec2d create_unit_vec(const double angle);

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, x);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, y);

  double angle() const { return atan2(y_, x_); }
  void normalize();

  double length() const;
  double length_sqr() const;
  double distance_to(const Vec2d& other) const;
  double distance_sqr_to(const Vec2d& other) const;
  template <typename T>
  double distance_to(const T& other) const {
    return sqrt((x_ - other.x()) * (x_ - other.x()) +
                (y_ - other.y()) * (y_ - other.y()));
  }
  Vec2d& operator=(const Vec2d& rhs);
  Vec2d operator+(const Vec2d& other) const;
  Vec2d operator-(const Vec2d& other) const;
  Vec2d operator*(const double ratio) const;
  Vec2d operator/(const double ratio) const;
  Vec2d& operator+=(const Vec2d& other);
  Vec2d& operator-=(const Vec2d& other);
  Vec2d& operator*=(const double ratio);
  Vec2d& operator/=(const double ratio);
  bool operator==(const Vec2d& other) const;

  double cross_prod(const Vec2d& other) const;
  double inner_prod(const Vec2d& other) const;
  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void self_rotate(const double angle);
  std::string debug_string() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

Vec2d operator*(const double ratio, const Vec2d& vec);

}  // namespace planning
}  // namespace neodrive
