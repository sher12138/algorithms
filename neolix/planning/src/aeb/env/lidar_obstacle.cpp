#include "lidar_obstacle.h"
namespace neodrive {
namespace aeb {
namespace {
void GengeratePolygonFromPb(const PerceptionObstacle &obs,
                            std::shared_ptr<common::math::Polygon2d> &polygon) {
  std::vector<common::math::Vec2d> polygon_points;
  for (auto &each_pt : obs.polygon_point()) {
    polygon_points.emplace_back(each_pt.x(), each_pt.y());
  }
  polygon.reset(new common::math::Polygon2d(polygon_points));
}
}  // namespace
Obstacle::Obstacle(const PerceptionObstacle &obs) {
  GengeratePolygonFromPb(obs, polygon_);
  vel_vector_.set_x(obs.velocity().x());
  vel_vector_.set_y(obs.velocity().y());
  vel_ = vel_vector_.length();
  vel_heading_ = std::atan2(obs.velocity().y(), obs.velocity().x());
  pos_.set_x(obs.position().x());
  pos_.set_y(obs.position().y());
}

void Obstacle::Update(const PerceptionObstacle &obs) {
  GengeratePolygonFromPb(obs, polygon_);
  vel_vector_.set_x(-obs.velocity().x());
  vel_vector_.set_y(-obs.velocity().y());
  pos_.set_x(obs.position().x());
  pos_.set_y(obs.position().y());
  vel_ = vel_vector_.length();
  vel_heading_ = std::atan2(obs.velocity().y(), obs.velocity().x());
  age_++;
  lost_count_ = 0;
  is_updated_ = true;
  ++update_count_;
}

void LidarObstacle::Update(const PerceptionObstacles &msg) {
  auto &aeb_config = config::AebConfig::Instance()->aeb_config();
  for (auto &each_obs : msg.perception_obstacle()) {
    if (each_obs.polygon_point_size() <= 3) continue;
    if (obstacles.count(each_obs.id()) == 0) {
      obstacles.emplace(each_obs.id(), std::make_shared<Obstacle>(each_obs));
    } else {
      obstacles[each_obs.id()]->Update(each_obs);
    }
  }
  auto iter = obstacles.begin();
  while (iter != obstacles.end()) {
    if (!iter->second->is_updated_) iter->second->lost_count_++;
    if (iter->second->lost_count_ > aeb_config.lidar_obstacle_lost_threshold)
      iter = obstacles.erase(iter);
    else
      iter++;
  }
}
}  // namespace aeb
}  // namespace neodrive