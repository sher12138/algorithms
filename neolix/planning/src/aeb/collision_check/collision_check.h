#pragma once
namespace neodrive {
namespace aeb {
class CollisionCheck {
 public:
  void Update();
  void VoteCollionCheck(bool imu_collision_check = false);

 private:
  bool collision_risk_{false};
  double last_imu_trigger_t{0.0};
  double last_collision_check_t{0.0};
};
}  // namespace aeb
}  // namespace neodrive