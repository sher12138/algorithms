#pragma once
#include "common/aeb_type.h"
#include "math/polygon2d.h"

namespace neodrive {
namespace aeb {
class Obstacle {
 public:
  Obstacle(const PerceptionObstacle &obs);
  void Update(const PerceptionObstacle &obs);

 public:
  int age_{0};
  int lost_count_{0};
  int update_count_{0};
  bool is_updated_{true};
  common::math::Vec2d pos_;
  double vel_{0.0};
  common::math::Vec2d vel_vector_;
  double vel_heading_{0.0};
  double relative_speed_{0.0};
  double pre_distance_{0.0};
  double distance_{0.0};
  std::shared_ptr<common::math::Polygon2d> polygon_{nullptr};
};

class LidarObstacle {
 public:
  LidarObstacle() {}
  ~LidarObstacle() { obstacles.clear(); }
  void Update(const PerceptionObstacles &msg);

 public:
  std::unordered_map<int, std::shared_ptr<Obstacle>> obstacles;
};
}  // namespace aeb
}  // namespace neodrive