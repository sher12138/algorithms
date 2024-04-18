#pragma once
#include <math.h>

#include "angles/angles.h"
#include "common_geometry.pb.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
namespace neodrive {
namespace planning_rl {
using neodrive::global::common::Quaternion;
template <typename T1, typename T2>
inline double Distance2d(const T1 &pt1, const T2 &pt2) {
  return std::sqrt(std::pow(pt1.x() - pt2.x(), 2) +
                   std::pow(pt1.y() - pt2.y(), 2));
}

template <typename T>
inline double Distance2d(const T x1, const T y1, const T x2, const T y2) {
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

}  // namespace planning_rl
}  // namespace neodrive
