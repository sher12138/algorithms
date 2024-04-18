#pragma once
#include <math.h>

#include "angles/angles.h"
#include "common_geometry.pb.h"
#include "math/vec2d.h"
#include "odometry.pb.h"

namespace neodrive {
namespace common {
using neodrive::common::angles::normalize_angle;
using neodrive::global::common::Point2D;
using neodrive::global::common::Point3D;
using neodrive::global::common::Pose;
using neodrive::global::common::Quaternion;
/**
 * @brief calculate yaw angle of point (x,y)
 * @param x,y: point pose
 * @return yaw angle from (0,0) to (x,y)
 */
static inline double YawAngle(const double &x, const double &y) {
  return normalize_angle(atan2(y, x));
}

static inline double GetYawFromQuaternion(const Quaternion &q) {
  double yaw;
  double x = q.qx();
  double y = q.qy();
  double z = q.qz();
  double w = q.qw();
  double sqw = w * w;
  double sqx = x * x;
  double sqy = y * y;
  double sqz = z * z;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw);

  if (sarg <= -0.99999) {
    yaw = -2 * atan2(y, x);
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(y, x);
  } else {
    yaw = atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz);
  }
  return normalize_angle(yaw);
}

/**
 * @brief calculate yaw angle of geometry pose
 * @param pose: geometry_pose
 * @return yaw angle of the pose
 */
static inline double GetYawFromPose(const Pose &pose) {
  return GetYawFromQuaternion(pose.orientation());
}

static inline double GetPitchFromQuaternion(const Quaternion &q) {
  return asin(2.0 * (q.qw() * q.qx() + q.qy() * q.qz()));
}

static inline double GetPitchFromPose(const Pose &pose) {
  return GetPitchFromQuaternion(pose.orientation());
}

static inline void CreateQuaternionFromYaw(Quaternion &q, const double &yaw) {
  q.set_qx(0.);
  q.set_qy(0.);
  q.set_qz(sin(yaw * 0.5));
  q.set_qw(cos(yaw * 0.5));
}

static inline void CreateQuaternionFromYaw(Quaternion *q, const double &yaw) {
  q->set_qx(0.);
  q->set_qy(0.);
  q->set_qz(sin(yaw * 0.5));
  q->set_qw(cos(yaw * 0.5));
}

static inline void CreateQuaternionFromYaw(Pose &pose, const double &yaw) {
  auto q = pose.mutable_orientation();
  q->set_qx(0.);
  q->set_qy(0.);
  q->set_qz(sin(yaw * 0.5));
  q->set_qw(cos(yaw * 0.5));
}

static inline void CreateQuaternionFromYaw(Pose *pose, const double &yaw) {
  auto q = pose->mutable_orientation();
  q->set_qx(0.);
  q->set_qy(0.);
  q->set_qz(sin(yaw * 0.5));
  q->set_qw(cos(yaw * 0.5));
}

/*****************************************************************************/
/******  transform point from world coordinate to relative coordinate  *******/
/*****************************************************************************/
template <typename T>
static inline void ConvertToRelativeCoordinate(const T &pt_in_world,
                                               const T &origin_in_world,
                                               T &pt_in_relative) {
  double diff_x = pt_in_world.x() - origin_in_world.x();
  double diff_y = pt_in_world.y() - origin_in_world.y();
  double origin_yaw = origin_in_world.z();
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  pt_in_relative.set_x(diff_x * cosr + diff_y * sinr);
  pt_in_relative.set_y(diff_y * cosr - diff_x * sinr);
  pt_in_relative.set_z(normalize_angle(pt_in_world.z() - origin_yaw));
}

template <typename T1, typename T2, typename T3>
static inline void ConvertToRelativeCoordinate2D(const T1 &pt_in_world,
                                                 const T2 &origin_in_world,
                                                 T3 &pt_in_relative) {
  double diff_x = pt_in_world.x() - origin_in_world.x();
  double diff_y = pt_in_world.y() - origin_in_world.y();
  double origin_yaw = origin_in_world.z();
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  pt_in_relative.set_x(diff_x * cosr + diff_y * sinr);
  pt_in_relative.set_y(diff_y * cosr - diff_x * sinr);
}

template <typename T1, typename T2, typename T3>
static inline void ConvertToRelativeCoordinate2D(const T1 &pt_in_world,
                                                 const T2 &origin_in_world,
                                                 const double origin_yaw,
                                                 T3 &pt_in_relative) {
  double diff_x = pt_in_world.x() - origin_in_world.x();
  double diff_y = pt_in_world.y() - origin_in_world.y();
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  pt_in_relative.set_x(diff_x * cosr + diff_y * sinr);
  pt_in_relative.set_y(diff_y * cosr - diff_x * sinr);
}

template <typename T>
static inline void ConvertToWorldCoordinate(const T &pt_in_relative,
                                            const T &origin_in_world,
                                            T &pt_in_world) {
  double origin_yaw = origin_in_world.z();
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  double diff_x = pt_in_relative.x() * cosr - pt_in_relative.y() * sinr;
  double diff_y = pt_in_relative.x() * sinr + pt_in_relative.y() * cosr;
  pt_in_world.set_x(diff_x + origin_in_world.x());
  pt_in_world.set_y(diff_y + origin_in_world.y());
  pt_in_world.set_z(normalize_angle(pt_in_relative.z() + origin_yaw));
}

template <typename T>
static inline void ConvertToWorldCoordinate(const T &pt_in_relative,
                                            const T &origin_in_world,
                                            const double origin_yaw,
                                            T &pt_in_world) {
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  double diff_x = pt_in_relative.x() * cosr - pt_in_relative.y() * sinr;
  double diff_y = pt_in_relative.x() * sinr + pt_in_relative.y() * cosr;
  pt_in_world.set_x(diff_x + origin_in_world.x());
  pt_in_world.set_y(diff_y + origin_in_world.y());
  pt_in_world.set_z(normalize_angle(pt_in_relative.z() + origin_yaw));
}

static inline void ConvertToWorldCoordinate(
    const common::math::Vec2d &pt_in_relative,
    const common::math::Vec2d &origin_in_world, const double origin_yaw,
    common::math::Vec2d &pt_in_world) {
  double cosr = cos(origin_yaw);
  double sinr = sin(origin_yaw);
  double diff_x = pt_in_relative.x() * cosr - pt_in_relative.y() * sinr;
  double diff_y = pt_in_relative.x() * sinr + pt_in_relative.y() * cosr;
  pt_in_world.set_x(diff_x + origin_in_world.x());
  pt_in_world.set_y(diff_y + origin_in_world.y());
}
}  // namespace common
}  // namespace neodrive