#pragma once

#include <memory>

#include "common/macros.h"
#include "cyber.h"
#include "cyber/common/memory_pool.h"
#include "node/writer.h"
#include "world_model/common/world_model_context.h"
#include "world_model/common/world_model_type.h"
#include "world_model/config/world_model_config.h"

namespace neodrive {
namespace world_model {

class CoordinateTransform {
  DECLARE_SINGLETON(CoordinateTransform);
  friend class CoordinateTransformTest;

 public:
  bool Init(std::shared_ptr<neodrive::cyber::Node> &node);
  void UpdateBaselinkPoseInUtm(const LocalizationEstimate &utm_pose_msg,
                               const Imu &imu_msg,
                               const LocalizationErrorCode &loc_error_code_msg,
                               const DRResult &dr_msg);
  void UpdateTwistInUtm(const LocalizationEstimate &utm_pose_msg,
                        const DRResult &odom_pose_msg, const Imu &imu_msg);
  void UpdateBaselinkPoseInOdom(const DRResult &odom_pose_msg,
                                const Imu &imu_msg);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::shared_ptr<neodrive::cyber::Node>,
                                   node);

 private:
  void SetTwistMsg(const Eigen::Vector3d &base_link_linear_velocity,
                   const Eigen::Vector3d &base_link_angular_velocity,
                   const Eigen::Vector3d &base_link_linear_acc,
                   TwistStamped &target_twist);
  void GetBaseLinkTwistFromUtmPose(const LocalizationEstimate &utm_pose,
                                   TwistStamped &base_link_twist_from_utm);
  void GetBaseLinkTwistFromImu(const DRResult &dr_msg, const Imu &imu_msg,
                               TwistStamped &base_link_twist_from_imu);
  void GetBaseLinkPose(const LocalizationEstimate &utm_msg,
                       PoseStamped &base_link_in_utm_pose);
  void GetBaseLinkPose(const LocalizationEstimate &utm_msg,
                       TwistStamped &base_link_in_utm_twist);
  void GetBaseLinkPose(const DRResult &dr_msg,
                       PoseStamped &base_link_in_odom_pose);
  void GetUtmTwist(const LocalizationEstimate &utm_pose_msg,
                   TwistStamped &twist_base_link);
  void GetOdomTwist(const DRResult &odom_pose_msg,
                    TwistStamped &twist_base_link);
  void GetCenterPose(const LocalizationEstimate &utm_msg,
                     PoseStamped &center_in_utm_pose);
  void GetCenterPose(const DRResult &dr_msg, PoseStamped &center_in_odom_pose);

  template <typename T>
  void SetSequenceNumAndStartTime(const uint32_t sequence_num,
                                  const double start_time, T &target_msg);

 private:
  static constexpr double kVelocityDiffThreshold = 0.1;
  static constexpr double kAccelerateDiffThreshold = 0.1;
  static constexpr double kTimeDiffThreshold = 0.1;

  std::atomic<bool> initialized_{false};
  uint32_t sequence_num_;
  std::shared_ptr<neodrive::cyber::Node> node_;
  config::WorldModelConfig *world_model_config_;
  WorldModelContxt *wm_context_{WorldModelContxt::Instance()};
  double odom_pose_process_time_{0.};
  double utm_pose_process_time_{0.};
  double kinematics_estimate_process_time_{0.};

  std::shared_ptr<neodrive::cyber::Writer<TwistStamped>>
      twist_base_link_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<TwistStamped>>
      kinematics_estimation_base_link_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<PoseStamped>>
      pose_base_link_in_odometry_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<PoseStamped>>
      pose_base_link_in_utm_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<PoseStamped>>
      pose_center_in_odometry_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<PoseStamped>>
      pose_center_in_utm_writer_{nullptr};
  std::shared_ptr<neodrive::cyber::Writer<LocalizationStatus>>
      loc_status_writer_{nullptr};

  bool odometry_initialized_{false};
  neodrive::cyber::common::MemoryPool<PoseStamped>
      base_link_pose_in_utm_msg_pool_;
  neodrive::cyber::common::MemoryPool<PoseStamped>
      base_link_pose_in_odom_msg_pool_;
  neodrive::cyber::common::MemoryPool<LocalizationStatus> loc_status_msg_pool_;
  neodrive::cyber::common::MemoryPool<TwistStamped>
      base_link_twist_from_imu_msg_pool_;
  neodrive::cyber::common::MemoryPool<TwistStamped>
      base_link_kinematics_estimation_from_imu_msg_pool_;
};

template <typename T>
void CoordinateTransform::SetSequenceNumAndStartTime(
    const uint32_t sequence_num, const double start_time, T &target_msg) {
  target_msg.mutable_header()->set_sequence_num(sequence_num);
  target_msg.mutable_header()->set_start_time(start_time);
}

}  // namespace world_model
}  // namespace neodrive
