#include "ego_car.h"

#include "common/coordinate/coodrdinate_convertion.h"
namespace neodrive {
namespace aeb {
namespace {
common::math::Vec2d GetEgoCarBoundary(const common::math::Vec2d relative_pt,
                                      const common::math::Vec2d origin_pt,
                                      const double origin_yaw) {
  common::math::Vec2d pt_in_world;
  common::ConvertToWorldCoordinate(relative_pt, origin_pt, origin_yaw,
                                   pt_in_world);
  return pt_in_world;
}
}  // namespace
EgoCar::EgoCar() { ego_polygon_.reset(new common::math::Polygon2d()); }

void EgoCar::Update(const Chassis &msg) { speed_ = msg.speed_mps(); }

void EgoCar::Update(const LocalizationEstimate &msg) {
  auto &common_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  current_pos_.set_x(msg.pose().position().x());
  current_pos_.set_y(msg.pose().position().y());
  heading_ = common::GetYawFromQuaternion(msg.pose().orientation());
  std::vector<common::math::Vec2d> boundarys;
  boundarys.emplace_back(
      GetEgoCarBoundary({-common_config.width / 2.0,
                         common_config.length - common_config.imu_in_car_x},
                        current_pos_, heading_));
  boundarys.emplace_back(GetEgoCarBoundary(
      {-common_config.width / 2.0, -common_config.imu_in_car_x}, current_pos_,
      heading_));
  boundarys.emplace_back(GetEgoCarBoundary(
      {common_config.width / 2.0, -common_config.imu_in_car_x}, current_pos_,
      heading_));
  boundarys.emplace_back(
      GetEgoCarBoundary({common_config.width / 2.0,
                         common_config.length - common_config.imu_in_car_x},
                        current_pos_, heading_));
  ego_polygon_.reset(new common::math::Polygon2d(boundarys));
}
}  // namespace aeb
}  // namespace neodrive