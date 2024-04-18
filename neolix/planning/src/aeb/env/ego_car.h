#pragma once
#include "common/aeb_type.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
namespace neodrive {
namespace aeb {
class EgoCar {
 public:
  EgoCar();
  void Update(const LocalizationEstimate &msg);
  void Update(const Chassis &msg);

 public:
  common::math::Polygon2d &EgoPolygon() { return *ego_polygon_; }
  common::math::Vec2d &EgoPos() { return current_pos_; }
  double Heading() { return heading_; }
  double Speed() { return speed_; }

 private:
  std::shared_ptr<common::math::Polygon2d> ego_polygon_{nullptr};
  common::math::Vec2d current_pos_;
  double heading_{0.0};
  double speed_{0.0};
};
}  // namespace aeb
}  // namespace neodrive