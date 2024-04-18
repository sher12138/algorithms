#include "coordinate_transform_test.h"

#include <cmath>

#include "common/coordinate/coodrdinate_convertion.h"
#include "gtest/gtest.h"

namespace neodrive {
namespace world_model {

void CoordinateTransformTest::TestGetBaseLinkTwistFromUtmPose() {
  LocalizationEstimate utm_msg;
  utm_msg.mutable_pose()->mutable_position()->set_x(0.);
  utm_msg.mutable_pose()->mutable_position()->set_y(0.);
  utm_msg.mutable_pose()->mutable_position()->set_z(0.);
  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  0);
  LOG_DEBUG("utm_msg orientation: {} {}",
            utm_msg.pose().orientation().DebugString(),
            common::GetYawFromQuaternion(utm_msg.pose().orientation()));

  utm_msg.mutable_pose()->mutable_linear_velocity_vrf()->set_x(0.);
  utm_msg.mutable_pose()->mutable_linear_velocity_vrf()->set_y(1.);
  utm_msg.mutable_pose()->mutable_linear_velocity_vrf()->set_z(0.);
  utm_msg.mutable_pose()->mutable_angular_velocity_vrf()->set_x(0.);
  utm_msg.mutable_pose()->mutable_angular_velocity_vrf()->set_y(0.);
  utm_msg.mutable_pose()->mutable_angular_velocity_vrf()->set_z(0.);
  utm_msg.mutable_pose()->mutable_linear_acceleration_vrf()->set_x(0.);
  utm_msg.mutable_pose()->mutable_linear_acceleration_vrf()->set_y(3.);
  utm_msg.mutable_pose()->mutable_linear_acceleration_vrf()->set_z(9.8);
  TwistStamped base_link_twist;

  coordinate_transform_->GetBaseLinkTwistFromUtmPose(utm_msg, base_link_twist);
  LOG_DEBUG("base_link_twist: {}", base_link_twist.DebugString());
}

void CoordinateTransformTest::TestGetBaseLinkTwistFromImu() {
  DRResult dr_msg;
  dr_msg.set_vy(1.);
  dr_msg.mutable_odometry_pose()->mutable_position()->set_x(0.);
  dr_msg.mutable_odometry_pose()->mutable_position()->set_y(0.);
  dr_msg.mutable_odometry_pose()->mutable_position()->set_z(0.);
  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), 0);
  LOG_DEBUG("dr_msg orientation: {} {}",
            dr_msg.odometry_pose().orientation().DebugString(),
            common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  Imu imu_msg;
  imu_msg.mutable_angular_velocity()->set_x(0.);
  imu_msg.mutable_angular_velocity()->set_y(0.);
  imu_msg.mutable_angular_velocity()->set_z(0.);
  imu_msg.mutable_linear_acceleration()->set_x(0.);
  imu_msg.mutable_linear_acceleration()->set_y(3.);
  imu_msg.mutable_linear_acceleration()->set_z(9.8);
  TwistStamped base_link_twist;
  coordinate_transform_->GetBaseLinkTwistFromImu(dr_msg, imu_msg,
                                                 base_link_twist);
  LOG_DEBUG("base_link_twist: {}", base_link_twist.DebugString());
}

void CoordinateTransformTest::TestGetBaseLinkPose1() {
  DRResult dr_msg;
  PoseStamped base_link_pose;
  dr_msg.mutable_odometry_pose()->mutable_position()->set_x(0.);
  dr_msg.mutable_odometry_pose()->mutable_position()->set_y(0.);
  dr_msg.mutable_odometry_pose()->mutable_position()->set_z(0.);
  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), 0);
  LOG_INFO("dr_msg orientation: {} {}",
           dr_msg.odometry_pose().orientation().DebugString(),
           common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(dr_msg, base_link_pose);
  LOG_INFO("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), 0.5 * M_PI);
  LOG_INFO("dr_msg orientation: {} {}",
           dr_msg.odometry_pose().orientation().DebugString(),
           common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(dr_msg, base_link_pose);
  LOG_INFO("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), M_PI);
  LOG_INFO("dr_msg orientation: {} {}",
           dr_msg.odometry_pose().orientation().DebugString(),
           common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(dr_msg, base_link_pose);
  LOG_INFO("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), -0.5 * M_PI);
  LOG_INFO("dr_msg orientation: {} {}",
           dr_msg.odometry_pose().orientation().DebugString(),
           common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(dr_msg, base_link_pose);
  LOG_INFO("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(
      dr_msg.mutable_odometry_pose()->mutable_orientation(), -1 * M_PI);
  LOG_INFO("dr_msg orientation: {} {}",
           dr_msg.odometry_pose().orientation().DebugString(),
           common::GetYawFromQuaternion(dr_msg.odometry_pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(dr_msg, base_link_pose);
  LOG_INFO("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));
}

void CoordinateTransformTest::TestGetBaseLinkPose2() {
  LocalizationEstimate utm_msg;
  PoseStamped base_link_pose;
  utm_msg.mutable_pose()->mutable_position()->set_x(0.);
  utm_msg.mutable_pose()->mutable_position()->set_y(0.);
  utm_msg.mutable_pose()->mutable_position()->set_z(0.);
  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  0);
  LOG_WARN("utm_msg orientation: {} {}",
           utm_msg.pose().orientation().DebugString(),
           common::GetYawFromQuaternion(utm_msg.pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(utm_msg, base_link_pose);
  LOG_WARN("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  0.5 * M_PI);
  LOG_WARN("utm_msg orientation: {} {}",
           utm_msg.pose().orientation().DebugString(),
           common::GetYawFromQuaternion(utm_msg.pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(utm_msg, base_link_pose);
  LOG_WARN("base_link_pose: {} {}", base_link_pose.DebugString(),
           common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  M_PI);
  LOG_WARN("utm_msg orientation: {} {}",
           utm_msg.pose().orientation().DebugString(),
           common::GetYawFromQuaternion(utm_msg.pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(utm_msg, base_link_pose);
  LOG_ERROR("base_link_pose: {} {}", base_link_pose.DebugString(),
            common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  -0.5 * M_PI);
  LOG_ERROR("utm_msg orientation: {} {}",
            utm_msg.pose().orientation().DebugString(),
            common::GetYawFromQuaternion(utm_msg.pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(utm_msg, base_link_pose);
  LOG_ERROR("base_link_pose: {} {}", base_link_pose.DebugString(),
            common::GetYawFromQuaternion(base_link_pose.pose().orientation()));

  common::CreateQuaternionFromYaw(utm_msg.mutable_pose()->mutable_orientation(),
                                  -1 * M_PI);
  LOG_ERROR("utm_msg orientation: {} {}",
            utm_msg.pose().orientation().DebugString(),
            common::GetYawFromQuaternion(utm_msg.pose().orientation()));
  coordinate_transform_->GetBaseLinkPose(utm_msg, base_link_pose);
  LOG_ERROR("base_link_pose: {} {}", base_link_pose.DebugString(),
            common::GetYawFromQuaternion(base_link_pose.pose().orientation()));
}

namespace {
TEST(TestLocalizationOdometryProbe, TestGetBaseLinkTwistFromUtmPose) {
  CoordinateTransformTest coord_trans_test;
  coord_trans_test.TestGetBaseLinkTwistFromUtmPose();
}
TEST(TestLocalizationOdometryProbe, TestGetBaseLinkTwistFromImu) {
  CoordinateTransformTest coord_trans_test;
  coord_trans_test.TestGetBaseLinkTwistFromImu();
}
TEST(TestLocalizationOdometryProbe, TestGetBaseLinkPose1) {
  CoordinateTransformTest coord_trans_test;
  coord_trans_test.TestGetBaseLinkPose1();
}
TEST(TestLocalizationOdometryProbe, TestGetBaseLinkPose2) {
  CoordinateTransformTest coord_trans_test;
  coord_trans_test.TestGetBaseLinkPose2();
}
}  // namespace

}  // namespace world_model
}  // namespace neodrive
