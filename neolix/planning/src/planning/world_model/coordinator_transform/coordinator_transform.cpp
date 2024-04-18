#include "coordinator_transform.h"

#include <time.h>

#include "common/coordinate/coodrdinate_convertion.h"
#include "common_config/config/common_config.h"
#include "neolix_common/math/trans_coord.h"

namespace neodrive {
namespace world_model {
using neodrive::global::localization::MsfSecurityLevel;
using neodrive::global::localization::MsfSecurityLevel_Name;

// PERFORMANCE_TEST_DEFINE(coordinator_transform);
using neodrive::cyber::Node;
using neodrive::cyber::Writer;

CoordinateTransform::CoordinateTransform() : sequence_num_(1) {}

bool CoordinateTransform::Init(std::shared_ptr<Node> &node) {
  if (initialized_) {
    return true;
  }
  if (node == nullptr) {
    LOG_ERROR("node is nullptr!!!");
    return false;
  }
  node_ = node;
  world_model_config_ = config::WorldModelConfig::Instance();
  auto &topics_config = neodrive::common::config::CommonConfig::Instance()
                            ->topics_config()
                            .topics;
  twist_base_link_writer_ = node_->CreateWriter<TwistStamped>(
      topics_config.at("twist_base_link").topic);
  kinematics_estimation_base_link_writer_ = node_->CreateWriter<TwistStamped>(
      topics_config.at("kinematics_estimation_base_link").topic);
  pose_base_link_in_odometry_writer_ = node_->CreateWriter<PoseStamped>(
      topics_config.at("pose_base_link_in_odometry").topic);
  pose_base_link_in_utm_writer_ = node_->CreateWriter<PoseStamped>(
      topics_config.at("pose_base_link_in_utm").topic);
  pose_center_in_odometry_writer_ = node_->CreateWriter<PoseStamped>(
      topics_config.at("pose_center_in_odometry").topic);
  pose_center_in_utm_writer_ = node_->CreateWriter<PoseStamped>(
      topics_config.at("pose_center_in_utm").topic);
  loc_status_writer_ = node_->CreateWriter<LocalizationStatus>(
      topics_config.at("localization_status").topic);
  initialized_ = true;
  return true;
}

void CoordinateTransform::GetOdomTwist(const DRResult &odom_pose_msg,
                                       TwistStamped &twist_base_link) {
  auto yaw =
      common::GetYawFromQuaternion(odom_pose_msg.odometry_pose().orientation());
  double spsi = std::sin(yaw);
  double cpsi = std::cos(yaw);
  double stheta = 0.0;
  double ctheta = 1.0;
  auto twist = twist_base_link.mutable_twist();
  double tmp = twist->linear().x();
  twist->mutable_linear()->set_x(-twist->linear().y());
  twist->mutable_linear()->set_y(tmp);

  tmp = twist->linear_acc().x();
  twist->mutable_linear_acc()->set_x(-twist->linear_acc().y());
  twist->mutable_linear_acc()->set_y(tmp);

  tmp = twist->angular().x();
  twist->mutable_angular()->set_x(-twist->angular().y());
  twist->mutable_angular()->set_y(tmp);

  Eigen::Vector3d angular_rate, odom_pose_lever, velocity_odom,
      acceleration_odom, velocity_addition;
  odom_pose_lever << 0, 0, 0;
  velocity_odom << cpsi * twist->linear().x() - spsi * twist->linear().y(),
      spsi * twist->linear().x() + cpsi * twist->linear().y(), 0.0;
  acceleration_odom << cpsi * twist->linear_acc().x() -
                           spsi * twist->linear_acc().y(),
      spsi * twist->linear_acc().x() + cpsi * twist->linear_acc().y(), 0.0;
  angular_rate << twist->angular().x() * (cpsi * ctheta) -
                      spsi * twist->angular().y(),
      twist->angular().x() * (spsi * ctheta) + cpsi * twist->angular().y(),
      -twist->angular().x() * stheta + twist->angular().z();
  LOG_INFO("ODOM converter:{}\t{},{},{},{},{},{},{},{},{}", yaw,
           angular_rate(0), angular_rate(1), angular_rate(2), velocity_odom(0),
           velocity_odom(1), velocity_odom(2), acceleration_odom(0),
           acceleration_odom(1), acceleration_odom(2));

  velocity_addition = angular_rate.cross(odom_pose_lever);
  velocity_odom = velocity_odom + velocity_addition;
  acceleration_odom = acceleration_odom + angular_rate.cross(velocity_addition);

  LOG_INFO("ODOM converter:0.0\t{},{},{},{},{},{},{},{},{}", angular_rate(0),
           angular_rate(1), angular_rate(2), velocity_odom(0), velocity_odom(1),
           velocity_odom(2), acceleration_odom(0), acceleration_odom(1),
           acceleration_odom(2));

  twist->mutable_linear()->set_x(velocity_odom(0));
  twist->mutable_linear()->set_y(velocity_odom(1));
  twist->mutable_linear()->set_z(velocity_odom(2));
  twist->mutable_angular()->set_x(angular_rate(0));
  twist->mutable_angular()->set_y(angular_rate(1));
  twist->mutable_angular()->set_z(angular_rate(2));
  twist->mutable_linear_acc()->set_x(acceleration_odom(0));
  twist->mutable_linear_acc()->set_y(acceleration_odom(1));
  twist->mutable_linear_acc()->set_z(acceleration_odom(2));
}

void CoordinateTransform::GetUtmTwist(const LocalizationEstimate &utm_pose_msg,
                                      TwistStamped &twist_base_link) {
  auto yaw = common::GetYawFromQuaternion(utm_pose_msg.pose().orientation());
  double spsi = std::sin(yaw);
  double cpsi = std::cos(yaw);
  double stheta = 0.0;
  double ctheta = 1.0;
  auto twist = twist_base_link.mutable_twist();

  Eigen::Vector3d angular_rate, utm_pose_position, velocity_utm,
      acceleration_utm, velocity_addition;
  // utm_pose_position << utm_pose_msg.pose().position().x(),
  //    utm_pose_msg.pose().position().y(), utm_pose_msg.pose().position().z();
  utm_pose_position << 1.2, 0, 0;
  velocity_utm << cpsi * twist->linear().x() - spsi * twist->linear().y(),
      spsi * twist->linear().x() + cpsi * twist->linear().y(), 0.0;
  acceleration_utm << cpsi * twist->linear_acc().x() -
                          spsi * twist->linear_acc().y(),
      spsi * twist->linear_acc().x() + cpsi * twist->linear_acc().y(), 0.0;
  angular_rate << twist->angular().x() * (cpsi * ctheta) -
                      spsi * twist->angular().y(),
      twist->angular().x() * (spsi * ctheta) + cpsi * twist->angular().y(),
      -twist->angular().x() * stheta + twist->angular().z();
  LOG_INFO("UTM converter:{}\t{},{},{},{},{},{},{},{},{}", yaw, angular_rate(0),
           angular_rate(1), angular_rate(2), velocity_utm(0), velocity_utm(1),
           velocity_utm(2), acceleration_utm(0), acceleration_utm(1),
           acceleration_utm(2));

  velocity_addition = angular_rate.cross(utm_pose_position);
  velocity_utm = velocity_utm + velocity_addition;
  acceleration_utm = acceleration_utm + angular_rate.cross(velocity_addition);

  LOG_INFO("UTM converter:0.0\t{},{},{},{},{},{},{},{},{}", angular_rate(0),
           angular_rate(1), angular_rate(2), velocity_utm(0), velocity_utm(1),
           velocity_utm(2), acceleration_utm(0), acceleration_utm(1),
           acceleration_utm(2));

  twist->mutable_linear()->set_x(velocity_utm(0));
  twist->mutable_linear()->set_y(velocity_utm(1));
  twist->mutable_linear()->set_z(velocity_utm(2));
  twist->mutable_angular()->set_x(angular_rate(0));
  twist->mutable_angular()->set_y(angular_rate(1));
  twist->mutable_angular()->set_z(angular_rate(2));
  twist->mutable_linear_acc()->set_x(acceleration_utm(0));
  twist->mutable_linear_acc()->set_y(acceleration_utm(1));
  twist->mutable_linear_acc()->set_z(acceleration_utm(2));
}

void CoordinateTransform::UpdateBaselinkPoseInUtm(
    const LocalizationEstimate &utm_pose_msg, const Imu &imu_msg,
    const LocalizationErrorCode &loc_error_code_msg, const DRResult &dr_msg) {
  utm_pose_process_time_ = cyber::Time::Now().ToSecond();
  auto base_link_pose_in_utm = base_link_pose_in_utm_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(base_link_pose_in_utm);
  GetBaseLinkPose(utm_pose_msg, *base_link_pose_in_utm);
  SetSequenceNumAndStartTime(utm_pose_msg.header().sequence_num(),
                             utm_pose_msg.measurement_time(),
                             *base_link_pose_in_utm);
  LOG_DEBUG("imu_pose_in_utm: (x,y): ({} {}), yaw: {} pitch: {}",
            utm_pose_msg.pose().position().x(),
            utm_pose_msg.pose().position().y(),
            common::GetYawFromQuaternion(utm_pose_msg.pose().orientation()),
            common::GetPitchFromQuaternion(utm_pose_msg.pose().orientation()));
  LOG_DEBUG(
      "base_link_pose_in_utm: (x,y): ({} {}), yaw: {} pitch: {} process time: "
      "{}",
      base_link_pose_in_utm->pose().position().x(),
      base_link_pose_in_utm->pose().position().y(),
      common::GetYawFromPose(base_link_pose_in_utm->pose()),
      common::GetPitchFromPose(base_link_pose_in_utm->pose()),
      base_link_pose_in_utm->header().timestamp_sec() - utm_pose_process_time_);
  pose_base_link_in_utm_writer_->Write(base_link_pose_in_utm);

  auto center_pose_in_utm = base_link_pose_in_utm_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(center_pose_in_utm);
  GetCenterPose(utm_pose_msg, *center_pose_in_utm);
  SetSequenceNumAndStartTime(utm_pose_msg.header().sequence_num(),
                             utm_pose_msg.measurement_time(),
                             *center_pose_in_utm);
  LOG_DEBUG(
      "center_pose_in_utm: (x,y): ({} {}), yaw: {} pitch: {} process time: "
      "{}",
      center_pose_in_utm->pose().position().x(),
      center_pose_in_utm->pose().position().y(),
      common::GetYawFromPose(center_pose_in_utm->pose()),
      common::GetPitchFromPose(center_pose_in_utm->pose()),
      center_pose_in_utm->header().timestamp_sec() - utm_pose_process_time_);
  pose_center_in_utm_writer_->Write(center_pose_in_utm);

  // update localization security level.'

  auto loc_status = loc_status_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(loc_status);
  loc_status->mutable_header()->set_timestamp_sec(
      imu_msg.header().timestamp_sec());
  SetSequenceNumAndStartTime(utm_pose_msg.header().sequence_num(),
                             imu_msg.measurement_time(), *loc_status);
  loc_status->mutable_sensor_status()->CopyFrom(utm_pose_msg.sensor_status());
  loc_status->mutable_msf_status()->CopyFrom(utm_pose_msg.msf_status());
  loc_status->set_odometry_initialized(odometry_initialized_);
  loc_status->set_odometry_predict(dr_msg.odometry_predict());
  loc_status->set_odometry_measurement_time(dr_msg.header().timestamp_sec());
  loc_status->set_odometry_sensor_error_code(dr_msg.sensor_error_code());
  loc_status->set_odometry_raw_veh_time(dr_msg.raw_veh_time());
  loc_status->set_odometry_raw_imu_time(dr_msg.raw_imu_time());
  loc_status->set_utm_measurement_time(utm_pose_msg.header().timestamp_sec());
  loc_status->set_utm_raw_veh_time(utm_pose_msg.raw_veh_time());
  loc_status->set_utm_raw_imu_time(utm_pose_msg.raw_imu_time());
  if (loc_error_code_msg.header().timestamp_sec() <
      loc_status->header().timestamp_sec() -
          world_model_config_->world_model_config()
              .localization_error_code_obsolete_time) {
    // localization error code msg is tool old, reset it
    loc_status->mutable_error_code()->Clear();
  } else {
    loc_status->mutable_error_code()->CopyFrom(loc_error_code_msg);
  }
  loc_status_writer_->Write(loc_status);
}

void CoordinateTransform::UpdateBaselinkPoseInOdom(
    const DRResult &odom_pose_msg, const Imu &imu_msg) {
  odometry_initialized_ = odom_pose_msg.odometry_initialized();
  odom_pose_process_time_ = cyber::Time::Now().ToSecond();
  auto base_link_twist_from_imu =
      base_link_twist_from_imu_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(base_link_twist_from_imu);
  GetBaseLinkTwistFromImu(odom_pose_msg, imu_msg, *base_link_twist_from_imu);
  LOG_INFO("{},{},{}", base_link_twist_from_imu->twist().linear().x(),
           base_link_twist_from_imu->twist().linear().y(),
           base_link_twist_from_imu->twist().linear().z());

  SetSequenceNumAndStartTime(odom_pose_msg.header().sequence_num(),
                             imu_msg.measurement_time(),
                             *base_link_twist_from_imu);
// For Time latency statisctis
#define SET_TIMECOLLECTOR(var, name, value) var->add_##name(value)
  SET_TIMECOLLECTOR(
      base_link_twist_from_imu->mutable_header()->mutable_time_collector(),
      loc_dr, odom_pose_msg.header().timestamp_sec());
  SET_TIMECOLLECTOR(
      base_link_twist_from_imu->mutable_header()->mutable_time_collector(),
      novatel_imu, imu_msg.header().timestamp_sec());
  twist_base_link_writer_->Write(base_link_twist_from_imu);

  auto base_link_kinematics_estimation_from_imu =
      base_link_kinematics_estimation_from_imu_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(base_link_kinematics_estimation_from_imu);
  GetBaseLinkTwistFromImu(odom_pose_msg, imu_msg,
                          *base_link_kinematics_estimation_from_imu);
  GetOdomTwist(odom_pose_msg, *base_link_kinematics_estimation_from_imu);
  SetSequenceNumAndStartTime(odom_pose_msg.header().sequence_num(),
                             imu_msg.measurement_time(),
                             *base_link_kinematics_estimation_from_imu);
  SET_TIMECOLLECTOR(base_link_kinematics_estimation_from_imu->mutable_header()
                        ->mutable_time_collector(),
                    loc_dr, odom_pose_msg.header().timestamp_sec());
  SET_TIMECOLLECTOR(base_link_kinematics_estimation_from_imu->mutable_header()
                        ->mutable_time_collector(),
                    novatel_imu, imu_msg.header().timestamp_sec());
  kinematics_estimation_base_link_writer_->Write(
      base_link_kinematics_estimation_from_imu);

  auto base_link_pose_in_odom = base_link_pose_in_odom_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(base_link_pose_in_odom);
  GetBaseLinkPose(odom_pose_msg, *base_link_pose_in_odom);
  SetSequenceNumAndStartTime(odom_pose_msg.header().sequence_num(),
                             odom_pose_msg.measurement_time(),
                             *base_link_pose_in_odom);
  SET_TIMECOLLECTOR(
      base_link_pose_in_odom->mutable_header()->mutable_time_collector(),
      loc_dr, odom_pose_msg.header().timestamp_sec());

  LOG_DEBUG("imu_pose_in_odom: (x,y): ({} {}), yaw: {}",
            odom_pose_msg.odometry_pose().position().x(),
            odom_pose_msg.odometry_pose().position().y(),
            common::GetYawFromQuaternion(
                odom_pose_msg.odometry_pose().orientation()));
  LOG_DEBUG("base_link_pose_in_odom: (x,y): ({} {}), yaw: {}, process time: {}",
            base_link_pose_in_odom->pose().position().x(),
            base_link_pose_in_odom->pose().position().y(),
            common::GetYawFromQuaternion(
                base_link_pose_in_odom->pose().orientation()),
            base_link_pose_in_odom->header().timestamp_sec() -
                odom_pose_process_time_);

  pose_base_link_in_odometry_writer_->Write(base_link_pose_in_odom);

  auto center_pose_in_odom = base_link_pose_in_odom_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(center_pose_in_odom);
  GetCenterPose(odom_pose_msg, *center_pose_in_odom);
  SetSequenceNumAndStartTime(odom_pose_msg.header().sequence_num(),
                             odom_pose_msg.measurement_time(),
                             *center_pose_in_odom);
  LOG_DEBUG(
      "center_pose_in_odom: (x,y): ({} {}), yaw: {}, process time: {}",
      center_pose_in_odom->pose().position().x(),
      center_pose_in_odom->pose().position().y(),
      common::GetYawFromQuaternion(center_pose_in_odom->pose().orientation()),
      center_pose_in_odom->header().timestamp_sec() - odom_pose_process_time_);
  pose_center_in_odometry_writer_->Write(center_pose_in_odom);
}

void CoordinateTransform::GetBaseLinkPose(const LocalizationEstimate &utm_msg,
                                          PoseStamped &base_link_in_utm_pose) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  auto &position = utm_msg.pose().position();
  auto &orientation = utm_msg.pose().orientation();
  Eigen::Vector3d imu_position =
      Eigen::Vector3d{position.x(), position.y(), position.z()};
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond{
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz()};
  Eigen::Quaterniond attitude_extrinc;
  auto base_link_position = common::math::GetGlobalPositionFromImu(
      imu_position, imu_attitude, common_config->base_link_to_imu_lever_arm());
  auto base_link_orientation = common::math::GetGlobalAttitudeFromImu(
      imu_attitude, common_config->base_link_to_imu_rotation(),
      utm_msg.pose().install_error_yaw(), utm_msg.pose().install_error_pitch());
  base_link_in_utm_pose.mutable_header()->set_timestamp_sec(
      utm_msg.header().timestamp_sec());
  auto mutable_position =
      base_link_in_utm_pose.mutable_pose()->mutable_position();
  mutable_position->set_x(base_link_position(0));
  mutable_position->set_y(base_link_position(1));
  mutable_position->set_z(base_link_position(2));
  auto mutable_orientation =
      base_link_in_utm_pose.mutable_pose()->mutable_orientation();
  mutable_orientation->set_qx(base_link_orientation.x());
  mutable_orientation->set_qy(base_link_orientation.y());
  mutable_orientation->set_qz(base_link_orientation.z());
  mutable_orientation->set_qw(base_link_orientation.w());
}

void CoordinateTransform::GetBaseLinkPose(
    const LocalizationEstimate &utm_msg, TwistStamped &base_link_in_utm_twist) {
  auto linear_acc =
      base_link_in_utm_twist.mutable_twist()->mutable_linear_acc();
  double heading = utm_msg.pose().heading();
  double acc_x =
      linear_acc->y() * std::cos(heading) + linear_acc->x() * std::sin(heading);
  double acc_y = -linear_acc->x() * std::cos(heading) +
                 linear_acc->y() * std::sin(heading);
  linear_acc->set_x(acc_x);
  linear_acc->set_y(acc_y);
}

void CoordinateTransform::GetCenterPose(const LocalizationEstimate &utm_msg,
                                        PoseStamped &center_in_utm_pose) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  auto &position = utm_msg.pose().position();
  auto &orientation = utm_msg.pose().orientation();
  Eigen::Vector3d imu_position =
      Eigen::Vector3d{position.x(), position.y(), position.z()};
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond{
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz()};
  Eigen::Quaterniond attitude_extrinc;
  auto center_position = common::math::GetGlobalPositionFromImu(
      imu_position, imu_attitude, common_config->center_to_imu_lever_arm());
  auto center_orientation = common::math::GetGlobalAttitudeFromImu(
      imu_attitude, common_config->center_to_imu_rotation(),
      utm_msg.pose().install_error_yaw(), utm_msg.pose().install_error_pitch());
  center_in_utm_pose.mutable_header()->set_timestamp_sec(
      utm_msg.header().timestamp_sec());
  auto mutable_position = center_in_utm_pose.mutable_pose()->mutable_position();
  mutable_position->set_x(center_position(0));
  mutable_position->set_y(center_position(1));
  mutable_position->set_z(center_position(2));
  auto mutable_orientation =
      center_in_utm_pose.mutable_pose()->mutable_orientation();
  mutable_orientation->set_qx(center_orientation.x());
  mutable_orientation->set_qy(center_orientation.y());
  mutable_orientation->set_qz(center_orientation.z());
  mutable_orientation->set_qw(center_orientation.w());
}
void CoordinateTransform::SetTwistMsg(
    const Eigen::Vector3d &base_link_linear_velocity,
    const Eigen::Vector3d &base_link_angular_velocity,
    const Eigen::Vector3d &base_link_linear_acc, TwistStamped &target_twist) {
  auto linear_vel = target_twist.mutable_twist()->mutable_linear();
  linear_vel->set_x(base_link_linear_velocity(0));
  linear_vel->set_y(base_link_linear_velocity(1));
  linear_vel->set_z(base_link_linear_velocity(2));

  auto linear_acc = target_twist.mutable_twist()->mutable_linear_acc();
  linear_acc->set_x(base_link_linear_acc(0));
  linear_acc->set_y(base_link_linear_acc(1));
  linear_acc->set_z(base_link_linear_acc(2));

  auto angular = target_twist.mutable_twist()->mutable_angular();
  angular->set_x(base_link_angular_velocity(0));
  angular->set_y(base_link_angular_velocity(1));
  angular->set_z(base_link_angular_velocity(2));
}

void CoordinateTransform::GetBaseLinkTwistFromUtmPose(
    const LocalizationEstimate &utm_pose,
    TwistStamped &base_link_twist_from_utm) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  Eigen::Vector3d vec_imu_linear_velocity_vrf;
  Eigen::Vector3d vec_imu_angular_velocity_vrf;
  Eigen::Vector3d vec_imu_linear_accelerator_vrf;

  Eigen::Vector3d base_link_linear_velocity;
  Eigen::Vector3d base_link_angular_velocity;
  Eigen::Vector3d base_link_linear_acc;

  auto &linear_velocity_vrf = utm_pose.pose().linear_velocity_vrf();
  auto &angular_velocity_vrf = utm_pose.pose().angular_velocity_vrf();
  auto &linear_acceleration_vrf = utm_pose.pose().linear_acceleration_vrf();
  auto &orientation = utm_pose.pose().orientation();
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond(
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

  vec_imu_linear_velocity_vrf << linear_velocity_vrf.x(),
      linear_velocity_vrf.y(), linear_velocity_vrf.z();
  vec_imu_angular_velocity_vrf << angular_velocity_vrf.x(),
      angular_velocity_vrf.y(), angular_velocity_vrf.z();
  vec_imu_linear_accelerator_vrf << linear_acceleration_vrf.x(),
      linear_acceleration_vrf.y(), linear_acceleration_vrf.z();
  base_link_linear_velocity = common::math::GetBaseLinkLinearVelocityFromImu(
      vec_imu_linear_velocity_vrf, vec_imu_angular_velocity_vrf,
      common_config->base_link_to_imu_lever_arm());
  base_link_angular_velocity = common::math::GetBaseLinkAngularVelocityFromImu(
      vec_imu_angular_velocity_vrf);
  base_link_linear_acc = common::math::GetBaseLinkLinearAcceleratorFromImu(
      vec_imu_linear_accelerator_vrf, vec_imu_angular_velocity_vrf,
      imu_attitude, common_config->base_link_to_imu_lever_arm());

  SetTwistMsg(base_link_linear_velocity, base_link_angular_velocity,
              base_link_linear_acc, base_link_twist_from_utm);
  base_link_twist_from_utm.mutable_header()->set_timestamp_sec(
      utm_pose.header().timestamp_sec());
}

void CoordinateTransform::GetBaseLinkTwistFromImu(
    const DRResult &dr_msg, const Imu &imu_msg,
    TwistStamped &base_link_twist_from_imu) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  Eigen::Vector3d vec_imu_linear_velocity_vrf;
  Eigen::Vector3d vec_imu_angular_velocity_vrf;
  Eigen::Vector3d vec_imu_linear_accelerator_vrf;

  Eigen::Vector3d base_link_linear_velocity;
  Eigen::Vector3d base_link_angular_velocity;
  Eigen::Vector3d base_link_linear_acc;
  auto &orientation = dr_msg.odometry_pose().orientation();
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond(
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

  base_link_linear_velocity << dr_msg.vy(), 0.0, 0.0;
  vec_imu_angular_velocity_vrf << imu_msg.angular_velocity().x(),
      imu_msg.angular_velocity().y(), imu_msg.angular_velocity().z();
  vec_imu_linear_accelerator_vrf << imu_msg.linear_acceleration().x(),
      imu_msg.linear_acceleration().y(), imu_msg.linear_acceleration().z();
  base_link_angular_velocity = common::math::GetBaseLinkAngularVelocityFromImu(
      vec_imu_angular_velocity_vrf);
  base_link_linear_acc = common::math::GetBaseLinkLinearAcceleratorFromImu(
      vec_imu_linear_accelerator_vrf, vec_imu_angular_velocity_vrf,
      imu_attitude, common_config->base_link_to_imu_lever_arm());
  SetTwistMsg(base_link_linear_velocity, base_link_angular_velocity,
              base_link_linear_acc, base_link_twist_from_imu);
  base_link_twist_from_imu.mutable_header()->set_timestamp_sec(
      imu_msg.header().timestamp_sec());
}

void CoordinateTransform::GetBaseLinkPose(const DRResult &dr_msg,
                                          PoseStamped &base_link_in_odom_pose) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  auto &position = dr_msg.odometry_pose().position();
  auto &orientation = dr_msg.odometry_pose().orientation();
  Eigen::Vector3d imu_position =
      Eigen::Vector3d{position.x(), position.y(), position.z()};
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond{
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz()};
  Eigen::Quaterniond attitude_extrinc;
  auto base_link_position = common::math::GetGlobalPositionFromImu(
      imu_position, imu_attitude, common_config->base_link_to_imu_lever_arm());
  auto base_link_orientation = common::math::GetGlobalAttitudeFromImu(
      imu_attitude, common_config->base_link_to_imu_rotation());
  base_link_in_odom_pose.mutable_header()->set_timestamp_sec(
      dr_msg.header().timestamp_sec());
  auto mutable_position =
      base_link_in_odom_pose.mutable_pose()->mutable_position();
  mutable_position->set_x(base_link_position(0));
  mutable_position->set_y(base_link_position(1));
  mutable_position->set_z(base_link_position(2));
  auto mutable_orientation =
      base_link_in_odom_pose.mutable_pose()->mutable_orientation();
  mutable_orientation->set_qx(base_link_orientation.x());
  mutable_orientation->set_qy(base_link_orientation.y());
  mutable_orientation->set_qz(base_link_orientation.z());
  mutable_orientation->set_qw(base_link_orientation.w());
}

void CoordinateTransform::GetCenterPose(const DRResult &dr_msg,
                                        PoseStamped &center_in_odom_pose) {
  auto common_config = neodrive::common::config::CommonConfig::Instance();
  auto &position = dr_msg.odometry_pose().position();
  auto &orientation = dr_msg.odometry_pose().orientation();
  Eigen::Vector3d imu_position =
      Eigen::Vector3d{position.x(), position.y(), position.z()};
  Eigen::Quaterniond imu_attitude = Eigen::Quaterniond{
      orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz()};
  Eigen::Quaterniond attitude_extrinc;
  auto center_position = common::math::GetGlobalPositionFromImu(
      imu_position, imu_attitude, common_config->center_to_imu_lever_arm());
  auto center_orientation = common::math::GetGlobalAttitudeFromImu(
      imu_attitude, common_config->center_to_imu_rotation());
  center_in_odom_pose.mutable_header()->set_timestamp_sec(
      dr_msg.header().timestamp_sec());
  auto mutable_position =
      center_in_odom_pose.mutable_pose()->mutable_position();
  mutable_position->set_x(center_position(0));
  mutable_position->set_y(center_position(1));
  mutable_position->set_z(center_position(2));
  auto mutable_orientation =
      center_in_odom_pose.mutable_pose()->mutable_orientation();
  mutable_orientation->set_qx(center_orientation.x());
  mutable_orientation->set_qy(center_orientation.y());
  mutable_orientation->set_qz(center_orientation.z());
  mutable_orientation->set_qw(center_orientation.w());
}
}  // namespace world_model
}  // namespace neodrive
