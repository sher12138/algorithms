#include "env.h"
namespace neodrive {
namespace aeb {
void Environment::UpdateLidarObstacles(const PerceptionObstacles &msg) {
  lidar_obstacles_.Update(msg);
}
void Environment::UpdateLocalization(const LocalizationEstimate &msg) {
  ego_car_.Update(msg);
}
void Environment::UpdateChassis(const Chassis &msg) { ego_car_.Update(msg); }
}  // namespace aeb
}  // namespace neodrive