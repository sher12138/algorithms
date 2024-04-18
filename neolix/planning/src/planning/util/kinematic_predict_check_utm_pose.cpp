#pragma once

#include "kinematic_predict_check_utm_pose.h"

namespace neodrive {
namespace planning {

KinematicPredictCheckUtmPose::KinematicPredictCheckUtmPose() {
  data_center_ = DataCenter::Instance();
}

bool KinematicPredictCheckUtmPose::Check() {
  if (data_center_->last_frame() == nullptr) {
    return true;
  }

  auto& vehicle_state_utm = data_center_->vehicle_state_utm();
  auto& vehicle_state_odometry = data_center_->vehicle_state_odometry();
  auto& last_vehicle_state_utm = data_center_->last_vehicle_state_utm();
  auto& last_vehicle_state_odometry =
      data_center_->last_vehicle_state_odometry();

  double delta_dis_utm = std::sqrt(
      std::pow(vehicle_state_utm.X() - last_vehicle_state_utm.X(), 2) +
      std::pow(vehicle_state_utm.Y() - last_vehicle_state_utm.Y(), 2));
  double delta_dis_odometry = std::sqrt(
      std::pow(vehicle_state_odometry.X() - last_vehicle_state_odometry.X(),
               2) +
      std::pow(vehicle_state_odometry.Y() - last_vehicle_state_odometry.Y(),
               2));
  LOG_INFO("delta_dis_utm/odometry: {:.3f}, {:.3f}", delta_dis_utm,
           delta_dis_odometry);

  // for current frame and last frame
  if (delta_dis_utm > delta_dis_odometry + 0.5) {
    return false;
  }
  // for history frame

  return true;
}

}  // namespace planning
}  // namespace neodrive