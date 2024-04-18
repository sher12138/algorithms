#pragma once
#include "common/aeb_type.h"
#include "ego_car.h"
#include "lidar_obstacle.h"
namespace neodrive {
namespace aeb {
class Environment {
 public:
  void UpdateLidarObstacles(const PerceptionObstacles &msg);
  void UpdateLocalization(const LocalizationEstimate &msg);
  void UpdateChassis(const Chassis &msg);

 public:
  LidarObstacle lidar_obstacles_;
  EgoCar ego_car_;
};
}  // namespace aeb
}  // namespace neodrive