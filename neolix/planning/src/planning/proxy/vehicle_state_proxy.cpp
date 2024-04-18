#include "vehicle_state_proxy.h"

#include "common/common_macros.h"
#include "common/data_center/data_center.h"
#include "common/math/double.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common_config/config/common_config.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/math/common/euler_angles_zxy.h"

namespace neodrive {
namespace planning {
using neodrive::common::config::AutoEgoCarConfig;
using neodrive::global::status::Chassis;
using neodrive::global::status::DrivingMode_Name;
using neodrive::global::status::GearPosition_Name;

void VehicleStateProxy::SetVehicleStatus(const ChassisShrPtr &chassis) {
  SET_PTR(chassis_, chassis, "SetVehicleStatus::ChassisShrPtr");
  chassis_valid_ = true;
  LOG_DEBUG("chassis valid: {}", chassis_valid_);
}

void VehicleStateProxy::SetVehicleStatus(const TwistStampedShrPtr &kinematics) {
  SET_PTR(kinematics_, kinematics, "SetVehicleStatus::kinematicsShrPtr");
  kinematics_valid_ = true;
  LOG_INFO("kinematics valid: {}", kinematics_valid_);
}

void VehicleStateProxy::SetVehicleStatus(const PoseStampedShrPtr &center_pose) {
  SET_PTR(center_pose_, center_pose, "SetVehicleStatus::PoseStampedShrPtr");
  center_pose_valid_ = true;
  LOG_DEBUG("center valid: {}", center_pose_valid_);
}

void VehicleStateProxy::SetVehicleStatus(const PoseStampedShrPtr &pose,
                                         const TwistStampedShrPtr &twist) {
  if (!chassis_valid_) {
    LOG_ERROR("chassis not valid! ignore msg.");
    return;
  }
  SET_PTR(pose_, pose, "SetVehicleStatus::PoseStampedShrPtr");
  pose_valid_ = true;
  LOG_DEBUG("pose valid: {}", pose_valid_);

  SET_PTR(twist_, twist, "SetVehicleStatus::TwistStampedShrPtr");
  twist_valid_ = true;
  LOG_DEBUG("twist valid: {}", twist_valid_);

  UpdateInnerStatus();
}

bool VehicleStateProxy::IsValid() const {
  if (!chassis_valid_ || !pose_valid_ || !twist_valid_ ||
      (!center_pose_valid_ && DataCenter::Instance()->use_center_pose())) {
    return false;
  }
  check_proto_nan(chassis_->steering_percentage());
  check_proto_nan(chassis_->speed_mps());
  check_proto_inf(chassis_->steering_percentage());
  check_proto_inf(chassis_->speed_mps());

  if (DataCenter::Instance()->use_center_pose()) {
    check_proto_nan(center_pose_->pose().position().x());
    check_proto_nan(center_pose_->pose().position().y());
    check_proto_nan(center_pose_->pose().position().z());
  }
  check_proto_nan(pose_->pose().position().x());
  check_proto_nan(pose_->pose().position().y());
  check_proto_nan(pose_->pose().position().z());
  check_proto_nan(pose_->pose().orientation().qx());
  check_proto_nan(pose_->pose().orientation().qy());
  check_proto_nan(pose_->pose().orientation().qz());
  check_proto_nan(pose_->pose().orientation().qw());
  check_proto_inf(pose_->pose().position().x());
  check_proto_inf(pose_->pose().position().y());
  check_proto_inf(pose_->pose().position().z());
  check_proto_inf(pose_->pose().orientation().qx());
  check_proto_inf(pose_->pose().orientation().qy());
  check_proto_inf(pose_->pose().orientation().qz());
  check_proto_inf(pose_->pose().orientation().qw());

  check_proto_nan(twist_->twist().linear().x());
  check_proto_nan(twist_->twist().linear().y());
  check_proto_nan(twist_->twist().linear().z());
  check_proto_nan(twist_->twist().linear_acc().x());
  check_proto_nan(twist_->twist().linear_acc().y());
  check_proto_nan(twist_->twist().linear_acc().z());
  check_proto_nan(twist_->twist().angular().z());
  check_proto_inf(twist_->twist().linear().x());
  check_proto_inf(twist_->twist().linear().y());
  check_proto_inf(twist_->twist().linear().z());
  check_proto_inf(twist_->twist().linear_acc().x());
  check_proto_inf(twist_->twist().linear_acc().y());
  check_proto_inf(twist_->twist().linear_acc().z());
  check_proto_inf(twist_->twist().angular().z());

  return true;
}

void VehicleStateProxy::SetTrajPitch(const double traj_pitch) {
  traj_pitch_ = traj_pitch;
}

void VehicleStateProxy::UpdateInnerStatus() {
  auto &ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  linear_velocity_ = twist_->twist().linear().x();
  linear_acceleration_ = twist_->twist().linear_acc().x();
  imu_acc_ = {-twist_->twist().linear_acc().y(),
              twist_->twist().linear_acc().x()};

  const auto &quaternion = pose_->pose().orientation();
  heading_ = std::atan2(2.0 * (quaternion.qw() * quaternion.qz() +
                               quaternion.qx() * quaternion.qy()),
                        1.0 - 2.0 * (quaternion.qy() * quaternion.qy() +
                                     quaternion.qz() * quaternion.qz()));
  heading_ = normalize_angle(heading_);

  EulerAnglesZXY<double> q_to_euler(quaternion.qw(), quaternion.qx(),
                                    quaternion.qy(), quaternion.qz());
  pitch_ = q_to_euler.pitch;
  roll_ = q_to_euler.roll;

  if (std::isnan(chassis_->steering_percentage())) {
    LOG_ERROR("Missing chassis steering percentage info!");
    if (Double::compare(LinearVelocity(), 0.0) == 0) {
      curvature_ = 0.0;
    } else {
      curvature_ = twist_->twist().angular().z() / LinearVelocity();
    }
    LOG_INFO("Vehicle curvature is curv: {}, no steer_percent", curvature_);
  } else {
    double steer = static_cast<double>(chassis_->steering_percentage());
    curvature_ = std::tan(steer / 100.0 * ego_car_config.max_steer_angle /
                          ego_car_config.steer_ratio) /
                 ego_car_config.wheel_base;
    LOG_DEBUG(
        "Vehicle curvature is curv: {}, steer_percent: {}, max_steer_angle "
        "{}, steer_ratio {}, wheel_base {}",
        curvature_, steer, ego_car_config.max_steer_angle,
        ego_car_config.steer_ratio, ego_car_config.wheel_base);
  }
}
double VehicleStateProxy::CenterX() const {
  return center_pose_->pose().position().x();
}
double VehicleStateProxy::CenterY() const {
  return center_pose_->pose().position().y();
}
double VehicleStateProxy::CenterZ() const {
  return center_pose_->pose().position().z();
}
double VehicleStateProxy::X() const {
  if (DataCenter::Instance()->use_center_pose()) {
    return center_pose_->pose().position().x();
  } else {
    return pose_->pose().position().x();
  }
}
double VehicleStateProxy::Y() const {
  if (DataCenter::Instance()->use_center_pose()) {
    return center_pose_->pose().position().y();
  } else {
    return pose_->pose().position().y();
  }
}
double VehicleStateProxy::Z() const {
  if (DataCenter::Instance()->use_center_pose()) {
    return center_pose_->pose().position().z();
  } else {
    return pose_->pose().position().z();
  }
}
double VehicleStateProxy::LinearVelocity() const {
  if (FLAGS_planning_use_chassis_speed_instead_of_localization_speed) {
    return std::abs(static_cast<double>(chassis_->speed_mps()));
  } else {
    return std::abs(linear_velocity_);
  }
}
double VehicleStateProxy::AbsoluteLinearVelocity() const {
  return std::fabs(LinearVelocity());
}
double VehicleStateProxy::Velocity3dX() const {
  return twist_->twist().linear().x() * std::cos(heading_);
}
double VehicleStateProxy::Velocity3dY() const {
  return twist_->twist().linear().x() * std::sin(heading_);
}
double VehicleStateProxy::Velocity3dZ() const {
  return twist_->twist().linear().z();
}
double VehicleStateProxy::Heading() const { return heading_; }
double VehicleStateProxy::Pitch() const { return pitch_; }
double VehicleStateProxy::TrajPitch() const { return traj_pitch_; }
double VehicleStateProxy::Roll() const { return roll_; }
double VehicleStateProxy::LinearAcceleration() const {
  return linear_acceleration_;
}
std::array<double, 2> VehicleStateProxy::ImuAcceleration() const {
  return imu_acc_;
}
double VehicleStateProxy::Curvature() const { return curvature_; }
double VehicleStateProxy::Timestamp() const {
  return pose_->header().timestamp_sec();
}
double VehicleStateProxy::SteerPercent() const {
  return static_cast<double>(chassis_->steering_percentage());
}
neodrive::global::status::GearPosition VehicleStateProxy::Gear() const {
  return chassis_->gear_location();
}
neodrive::global::status::DrivingMode VehicleStateProxy::DrivingMode() const {
  return FLAGS_playing_record
             ? neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE
             : chassis_->driving_mode();
}
bool VehicleStateProxy::IsStopped() const {
  auto &ego_car_config =
      neodrive::common::config::CommonConfig::Instance()->ego_car_config();
  if (LinearVelocity() > ego_car_config.stop_vel_threshold) {
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace neodrive
