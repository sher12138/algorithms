//!
//! \file
//! \brief Defines the Vec2d class.
//!
#pragma once

#include <cmath>
#include <string>

//!
//! \namespace neodrive::prediction
//! \brief neodrive::prediction
//!
namespace neodrive {
namespace planning_rl {

constexpr double kMathEpsilon = 1e-10;

//!
//! \class Vec2d
//!
//! \brief Implements a class of 2-dimensional vectors.
//!
class Vec2d {
 public:
  //!
  //! \brief Constructor which takes x- and y-coordinates.
  //!
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  //!
  //! \brief Constructor returning the zero vector.
  //!
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  //!
  //! \brief Creates unit-vector with a given angle to the positive x semi-axis
  //!
  static Vec2d CreateUnitVec2d(const double angle);

  //!
  //! \brief Getter for x component
  //!
  double x() const { return x_; }

  //!
  //! \brief Getter for y component
  //!
  double y() const { return y_; }

  //!
  //! \brief Setter for x component
  //!
  void set_x(const double x) { x_ = x; }

  //!
  //! \brief Setter for y component
  //!
  void set_y(const double y) { y_ = y; }

  //!
  //! \brief Gets the length of the vector
  //!
  double Length() const;

  //!
  //! \brief Gets the squared length of the vector
  //!
  double LengthSquare() const;

  //!
  //! \brief Gets the angle between the vector and the positive x semi-axis
  //!
  double Angle() const;

  //!
  //! \brief Returns the unit vector that is co-linear with this vector
  //!
  void Normalize();

  //!
  //! \brief Returns the distance to the given vector
  //!
  double DistanceTo(const Vec2d &other) const;

  //!
  //! \brief Returns the squared distance to the given vector
  //!
  double DistanceSquareTo(const Vec2d &other) const;

  //!
  //! \brief Returns the "cross" product between these two Vec2d (non-standard).
  //!
  double CrossProd(const Vec2d &other) const;

  //!
  //! \brief Returns the inner product between these two Vec2d.
  //!
  double InnerProd(const Vec2d &other) const;

  //!
  //! \brief rotate the vector by angle.
  //!
  Vec2d rotate(const double angle) const;

  //!
  //! \brief rotate the vector itself by angle.
  //!
  void SelfRotate(const double angle);

  //!
  //! \brief Sums two Vec2d
  //!
  Vec2d operator+(const Vec2d &other) const;

  //!
  //! \brief Subtracts two Vec2d
  //!
  Vec2d operator-(const Vec2d &other) const;

  //!
  //! \brief Multiplies Vec2d by a scalar
  //!
  Vec2d operator*(const double ratio) const;

  //!
  //! \brief Divides Vec2d by a scalar
  //!
  Vec2d operator/(const double ratio) const;

  //!
  //! \brief Sums another Vec2d to the current one
  //!
  Vec2d &operator+=(const Vec2d &other);

  //!
  //! \brief Subtracts another Vec2d to the current one
  //!
  Vec2d &operator-=(const Vec2d &other);

  //!
  //! \brief Multiplies this Vec2d by a scalar
  //!
  Vec2d &operator*=(const double ratio);

  //!
  //! \brief Divides this Vec2d by a scalar
  //!
  Vec2d &operator/=(const double ratio);

  //!
  //! \brief Compares two Vec2d
  //!
  bool operator==(const Vec2d &other) const;

  //!
  //! \brief Returns a human-readable string representing this object
  //!
  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//!
//! \brief Multiplies the given Vec2d by a given scalar
//!
Vec2d operator*(const double ratio, const Vec2d &vec);

}  // namespace planning_rl
}  // namespace neodrive
