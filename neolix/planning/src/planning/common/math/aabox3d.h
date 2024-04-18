/// @file Define the 3d axis-aligned box
#pragma once

#include <vector>

#include "common/math/vec3d.h"

namespace neodrive {
namespace planning {

class AABox3d {
 public:
  /// Construct with center and l, w, h
  /// @param center Center of the aabox3d
  /// @param length Length
  /// @param width Width
  /// @param height Height
  AABox3d(const Vec3d& center, const double length, const double width,
          const double height);

  /// Construct with left-bottom and right-top point
  /// @param one_corner The point of (min_x, min_y, min_z)
  /// @param opponent_corner The point of (max_x, max_y, max_z)
  AABox3d(const Vec3d& one_corner, const Vec3d& opponent_corner);

  /// Construct with points, the aabox3d with cover all points
  /// @param points All points in aabox3d
  AABox3d(const std::vector<Vec3d>& points);

  /// Getter of the center
  const Vec3d& center() const;

  /// Getter
  double length() const;

  /// Getter
  double width() const;

  /// Getter
  double height() const;

  /// Getter
  double half_length() const;

  /// Getter
  double half_width() const;

  /// Getter
  double half_height() const;

  /// Getter
  double volume() const;

  /// Getter
  double min_x() const;

  /// Getter
  double min_y() const;

  /// Getter
  double min_z() const;

  /// Getter
  double max_x() const;

  /// Getter
  double max_y() const;

  /// Getter
  double max_z() const;

  // Get all corners in counter clockwise order, bottom to top
  void get_all_corners(std::vector<Vec3d>* const corners) const;

  /// Check if the point is in box
  /// @param point Point to check
  /// @return True of not
  bool is_point_in(const Vec3d& point) const;

  /// Get the distance to a point
  /// @param point Point to check
  /// @return Distance
  double distance_to(const Vec3d& point) const;

  /// Get the distance to a box
  /// @param box Box to check
  /// @return Distance
  double distance_to(const AABox3d& box) const;

  /// Check if another box has overlap with this
  /// @param box The Box to check
  /// @return True or not
  bool has_overlap(const AABox3d& box) const;

  /// Shift the center of AABox by the input vector.
  /// @param shift_vec Thift vector
  void shift(const Vec3d& shift_vec);

  /// Merge another box to this
  /// @param other_box The other box
  void merge_from(const AABox3d& other_box);

  /// Merge another point to this
  /// @param other_point The other point
  void merge_from(const Vec3d& other_point);

 private:
  Vec3d center_{0, 0, 0};

  double length_{0};
  double width_{0};
  double height_{0};

  double half_length_{0};
  double half_width_{0};
  double half_height_{0};
};

}  // namespace planning
}  // namespace neodrive
