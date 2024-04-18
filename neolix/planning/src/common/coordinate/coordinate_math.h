#pragma once
#include <math.h>

#include "angles/angles.h"
#include "common_geometry.pb.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
namespace neodrive {
namespace common {
using neodrive::global::common::Quaternion;

template <typename T1, typename T2>
double Distance2D(const T1 &pt1, const T2 &pt2) {
  return std::hypot(pt1.x() - pt2.x(), pt1.y() - pt2.y());
}

template <typename T1, typename T2>
double Distance3D(const T1 &pt1, const T2 &pt2) {
  return std::sqrt(std::pow(pt1.x() - pt2.x(), 2) +
                   std::pow(pt1.y() - pt2.y(), 2) +
                   std::pow(pt1.z() - pt2.z(), 2));
}

template <typename T>
inline double Distance2d(const T x1, const T y1, const T x2, const T y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

}  // namespace common
}  // namespace neodrive
