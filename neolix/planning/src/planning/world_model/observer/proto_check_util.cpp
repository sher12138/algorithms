#include "proto_check_util.h"

#include "common/common_macros.h"
#include "neolix_log.h"

namespace neodrive {
namespace world_model {

bool CheckProto(const LocalizationErrorCode &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);

  return true;
}

bool CheckProto(const DRResult &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, odometry_pose);
  check_proto(proto.odometry_pose(), position);
  check_proto(proto.odometry_pose().position(), x);
  check_proto(proto.odometry_pose().position(), y);
  check_proto(proto.odometry_pose().position(), z);
  check_proto_nan(proto.odometry_pose().position().x());
  check_proto_nan(proto.odometry_pose().position().y());
  check_proto_nan(proto.odometry_pose().position().z());
  check_proto(proto.odometry_pose(), orientation);
  check_proto(proto.odometry_pose().orientation(), qx);
  check_proto(proto.odometry_pose().orientation(), qy);
  check_proto(proto.odometry_pose().orientation(), qz);
  check_proto(proto.odometry_pose().orientation(), qw);
  check_proto_nan(proto.odometry_pose().orientation().qx());
  check_proto_nan(proto.odometry_pose().orientation().qy());
  check_proto_nan(proto.odometry_pose().orientation().qz());
  check_proto_nan(proto.odometry_pose().orientation().qw());
  check_proto(proto, odometry_twist);
  check_proto(proto.odometry_twist(), linear);
  check_proto(proto.odometry_twist(), angular);
  check_proto(proto.odometry_twist().linear(), x);
  check_proto(proto.odometry_twist().linear(), y);
  check_proto(proto.odometry_twist().angular(), z);
  check_proto_nan(proto.odometry_twist().linear().x());
  check_proto_nan(proto.odometry_twist().linear().y());
  check_proto_nan(proto.odometry_twist().angular().z());
  check_proto(proto, odometry_initialized);
  check_proto(proto, odometry_predict);
  return true;
}

bool CheckProto(const TwistStamped &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto.twist(), linear);
  check_proto(proto.twist(), linear_acc);
  check_proto(proto.twist(), angular);
  check_proto(proto.twist().linear(), x);
  check_proto(proto.twist().linear(), y);
  check_proto(proto.twist().linear(), z);
  check_proto(proto.twist().linear_acc(), x);
  check_proto(proto.twist().linear_acc(), y);
  check_proto(proto.twist().linear_acc(), z);
  check_proto(proto.twist().angular(), x);
  check_proto(proto.twist().angular(), y);
  check_proto(proto.twist().angular(), z);
  check_proto_nan(proto.twist().linear().x());
  check_proto_nan(proto.twist().linear().y());
  check_proto_nan(proto.twist().linear().z());
  check_proto_nan(proto.twist().linear_acc().x());
  check_proto_nan(proto.twist().linear_acc().y());
  check_proto_nan(proto.twist().linear_acc().z());
  check_proto_nan(proto.twist().angular().x());
  check_proto_nan(proto.twist().angular().y());
  check_proto_nan(proto.twist().angular().z());
  return true;
}

bool CheckProto(const LocalizationEstimate &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, pose);
  check_proto(proto.pose(), position);
  check_proto(proto.pose().position(), x);
  check_proto(proto.pose().position(), y);
  check_proto(proto.pose().position(), z);
  check_proto_nan(proto.pose().position().x());
  check_proto_nan(proto.pose().position().y());
  check_proto_nan(proto.pose().position().z());

  check_proto(proto.pose(), orientation);
  check_proto(proto.pose().orientation(), qx);
  check_proto(proto.pose().orientation(), qy);
  check_proto(proto.pose().orientation(), qz);
  check_proto(proto.pose().orientation(), qw);
  check_proto_nan(proto.pose().orientation().qx());
  check_proto_nan(proto.pose().orientation().qy());
  check_proto_nan(proto.pose().orientation().qz());
  check_proto_nan(proto.pose().orientation().qw());

  check_proto(proto.pose(), linear_velocity);
  check_proto(proto.pose().linear_velocity(), x);
  check_proto(proto.pose().linear_velocity(), y);
  check_proto(proto.pose().linear_velocity(), z);
  check_proto_nan(proto.pose().linear_velocity().x());
  check_proto_nan(proto.pose().linear_velocity().y());
  check_proto_nan(proto.pose().linear_velocity().z());

  check_proto(proto.pose(), linear_acceleration);
  check_proto(proto.pose().linear_acceleration(), x);
  check_proto(proto.pose().linear_acceleration(), y);
  check_proto_nan(proto.pose().linear_acceleration().x());
  check_proto_nan(proto.pose().linear_acceleration().y());

  check_proto(proto.pose(), angular_velocity);
  check_proto(proto.pose().angular_velocity(), z);
  check_proto_nan(proto.pose().angular_velocity().z());

  return true;
}

bool CheckProto(const Imu &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, measurement_time);
  check_proto(proto, measurement_span);

  check_proto(proto, linear_acceleration);
  check_proto(proto.linear_acceleration(), x);
  check_proto(proto.linear_acceleration(), y);
  check_proto(proto.linear_acceleration(), z);
  check_proto(proto, angular_velocity);
  check_proto(proto.angular_velocity(), x);
  check_proto(proto.angular_velocity(), y);
  check_proto(proto.angular_velocity(), z);

  check_proto(proto, time_diff);
  return true;
}

bool CheckProto(const Chassis &proto) {
  check_proto(proto, header);
  check_proto(proto.header(), timestamp_sec);
  check_proto(proto, speed_mps);
  check_proto(proto, throttle_percentage);

  check_proto(proto, brake_percentage);
  check_proto(proto, steering_percentage);
  check_proto(proto, driving_mode);
  check_proto(proto, error_code);
  check_proto(proto, gear_location);
  check_proto(proto, sysfaultlevel);
  check_proto(proto, current_vcu_mode);
  return true;
}

}  // namespace world_model
}  // namespace neodrive
