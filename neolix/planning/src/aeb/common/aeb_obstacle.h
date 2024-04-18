#pragma once
#include "Eigen/Dense"
#include "aeb_type.h"
#include "common/coordinate/coodrdinate_convertion.h"
namespace neodrive {
namespace aeb {
struct AebObstacle {
  int32_t id;
  Point2D local_pos;
  double linear_velocity;
  int active_count;
  std::vector<Point2D> local_polygon;
};
typedef std::shared_ptr<AebObstacle> AebObstaclePtr;
static inline void ConvertOdomObstacle2Imu(
    const global::common::Pose &vehicle_pose,
    const PerceptionObstacle &perception_obstacle, Point2D &local_pos,
    std::vector<Point2D> &local_polygon) {
  Eigen::Translation3d trans(vehicle_pose.position().x(),
                             vehicle_pose.position().y(),
                             vehicle_pose.position().z());
  Eigen::Quaterniond quat(
      vehicle_pose.orientation().qw(), vehicle_pose.orientation().qx(),
      vehicle_pose.orientation().qy(), vehicle_pose.orientation().qz());
  Eigen::Affine3d odom2imu = (trans * quat).inverse();
  for (int i = 0; i < perception_obstacle.polygon_point_size(); ++i) {
    auto &origin_pt = perception_obstacle.polygon_point(i);
    Eigen::Vector3d op(origin_pt.x(), origin_pt.y(), origin_pt.z());
    Eigen::Vector3d lp = odom2imu * op;
    auto &local_pt = local_polygon[i];
    local_pt.set_x(lp[0]);
    local_pt.set_y(lp[1]);
  }
  Eigen::Vector3d op(perception_obstacle.position().x(),
                     perception_obstacle.position().y(),
                     perception_obstacle.position().z());
  Eigen::Vector3d lp = odom2imu * op;
  local_pos.set_x(lp[0]);
  local_pos.set_y(lp[1]);
}
static inline void TransformImuToPnc(Point2D &pt) {
  auto origin_x = pt.x();
  pt.set_x(pt.y());
  pt.set_y(-origin_x);
}
template <typename PoseType>
static inline void GetLocalObstacle(
    const PoseType &vehicle_pose, const double origin_heading,
    const PerceptionObstacle &perception_obstacle,
    AebObstaclePtr aeb_obstacle) {
  Point3D vehicle_pose_point;
  vehicle_pose_point.set_x(vehicle_pose.position().x());
  vehicle_pose_point.set_y(vehicle_pose.position().y());
  vehicle_pose_point.set_z(origin_heading);
  double obj_vel_yaw = std::atan2(-perception_obstacle.velocity().x(),
                                  perception_obstacle.velocity().y());
  double diff_yaw = common::normalize_angle(obj_vel_yaw - origin_heading);
  LOG_DEBUG("id:{},obstacle yaw:{},origin yaw:{}", perception_obstacle.id(),
            obj_vel_yaw, origin_heading);
  double origin_vel =
      std::sqrt(std::pow(perception_obstacle.velocity().x(), 2) +
                std::pow(perception_obstacle.velocity().y(), 2));
  aeb_obstacle->linear_velocity = (std::fabs(diff_yaw) <= M_PI_4)
                                      ? origin_vel
                                      : origin_vel * std::cos(diff_yaw);
  LOG_DEBUG("obstacle vel:{},linear:{},diff_yaw:{}", origin_vel,
            aeb_obstacle->linear_velocity, diff_yaw);
  aeb_obstacle->local_polygon.resize(perception_obstacle.polygon_point_size());
  ConvertOdomObstacle2Imu(vehicle_pose, perception_obstacle,
                          aeb_obstacle->local_pos, aeb_obstacle->local_polygon);
  TransformImuToPnc(aeb_obstacle->local_pos);
  for (int i = 0; i < aeb_obstacle->local_polygon.size(); ++i) {
    TransformImuToPnc(aeb_obstacle->local_polygon[i]);
  }
  aeb_obstacle->id = perception_obstacle.id();
  auto& obstacles = AebContext::Instance()->environment.lidar_obstacles_.obstacles;
  if (obstacles.count(aeb_obstacle->id)) aeb_obstacle->active_count = obstacles.at(aeb_obstacle->id)->update_count_;
  else aeb_obstacle->active_count = 0;
}
}  // namespace aeb
}  // namespace neodrive