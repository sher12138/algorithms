#include "collision_check.h"

#include "common/aeb_context.h"
#include "common/aeb_utils.h"
#include "math/segment2d.h"
#include "math/vec2d.h"

namespace neodrive {
namespace aeb {
void CollisionCheck::Update() {
  auto &ego_car = AebContext::Instance()->environment.ego_car_;
  auto &obstacles =
      AebContext::Instance()->environment.lidar_obstacles_.obstacles;
  auto &aeb_config = config::AebConfig::Instance()->aeb_config();
  auto &ego_pos = ego_car.EgoPos();
  auto &ego_polygon = ego_car.EgoPolygon();
  if (ego_polygon.points().size() < 3) return;
  auto iter = obstacles.begin();
  while (iter != obstacles.end()) {
    auto &obs = *iter->second;
    if (obs.is_updated_ && obs.polygon_->points().size() >= 3) {
      common::math::Segment2d pos_seg(obs.pos_, ego_pos);
      obs.pre_distance_ = obs.distance_;
      obs.distance_ = ego_polygon.distance_to(*obs.polygon_);
      double obs_heading_diff = obs.vel_heading_ - pos_seg.heading();
      double ego_heading_diff = ego_car.Heading() - pos_seg.heading();
      obs.relative_speed_ = obs.vel_ * std::cos(obs_heading_diff) -
                            ego_car.Speed() * std::cos(ego_heading_diff);
      if (obs.pre_distance_ > obs.distance_ &&
          obs.distance_ < aeb_config.relative_distance_threshold &&
          std::fabs(obs.relative_speed_) >=
              aeb_config.relative_speed_threshold) {
        collision_risk_ = true;
        last_collision_check_t = cyber::Time::Now().ToSecond();
        LOG_INFO("obs:{},relative_distance:{},relative_vel:{}", iter->first,
                 obs.distance_, obs.relative_speed_);
      }
      obs.is_updated_ = false;
    }
    iter++;
  }
}

void CollisionCheck::VoteCollionCheck(bool imu_collision_check) {
  Update();
  auto &patrol_msg = AebContext::Instance()->patrol_msg;
  if (patrol_msg.is_updated && patrol_msg.ptr->is_collision()) {
    last_imu_trigger_t = cyber::Time::Now().ToSecond();
  }
  if (collision_risk_ && last_collision_check_t - last_imu_trigger_t < 1.0)
    AebContext::Instance()->is_collisioned = true;
  else
    AebContext::Instance()->is_collisioned = false;
  collision_risk_ = false;
}
}  // namespace aeb
}  // namespace neodrive