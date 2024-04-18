#pragma once

#include <vector>

#include "common/math/vec2d.h"

namespace neodrive {
namespace planning {

class Polygon_RELATION {
 public:
  // construct&copy construct
  Polygon_RELATION();
  Polygon_RELATION(const Polygon_RELATION& other);
  ~Polygon_RELATION();
  //=、==overload
  Polygon_RELATION& operator=(const Polygon_RELATION& other);
  bool operator==(const Polygon_RELATION& other);
  // set value
  void SetRect(double x1, double y1, double x2, double y2, double x3, double y3,
               double x4, double y4);
  void SetRectvertex(std::vector<Vec2d> vertex);
  // collision check, true: collided
  bool PointInPolygon(double x, double y);
  bool PolygonCollision(Polygon_RELATION& other);

 public:
  std::vector<Vec2d> vertex_;  // corner of rectangle
};
}  // namespace planning
}  // namespace neodrive
