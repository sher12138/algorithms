#pragma once

#include "obsmap.h"

#include <bitset>

#include "common/visualizer_event/visualizer_event.h"
#include "src/common/coordinate/coodrdinate_convertion.h"
#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {
namespace obsmap {

ObsMap::ObsMap(int size, bool inverse) : max_size_{size}, inverse_{inverse} {
  coordinate_transformations_["odom2veh"] = &ObsMap::Odom2Veh;
  coordinate_transformations_["veh2odom"] = &ObsMap::Veh2Odom;
  coordinate_transformations_["utm2veh"] = &ObsMap::Utm2Veh;
  coordinate_transformations_["veh2utm"] = &ObsMap::Veh2Utm;
  coordinate_transformations_["utm2odom"] = &ObsMap::Utm2Odom;
  coordinate_transformations_["odom2utm"] = &ObsMap::Odom2Utm;
}

void ObsMap::VisObstacles(int frame) {
  if (!FLAGS_planning_enable_vis_event) return;
  LOG_INFO("Visualizer finished");
}

ObsFrameContainer &ObsMap::operator[](int frame) {
  if (frame < 0 || frame >= size_) {
    LOG_ERROR("Exception: Array index is out of bounds in obsmap");
    return obs_frame_container_.front();
  }
  if (frame < size_) {
    return obs_frame_container_[inverse_ ? size_ - 1 - frame : frame];
  }
}

const ObsFrameContainer &ObsMap::At(int frame) const {
  if (frame < 0 || frame >= size_) {
    LOG_ERROR("Exception: Array index is out of bounds in obsmap");
    return obs_frame_container_.front();
  }
  if (frame < size_) {
    return obs_frame_container_[inverse_ ? size_ - 1 - frame : frame];
  }
}

void ObsMap::Update() {
  if (IsFull()) Pop();
  Push();
  LOG_DEBUG("ObsMap Size:{}, #num: {}", size_,
            obs_frame_container_.front().mutable_obs_polygon_tree()->Length());
}

void ObsMap::Update(ObsFrameContainer &currentFrame) {
  if (IsFull()) Pop();
  Push(currentFrame);
  LOG_DEBUG("ObsMap Size:{}, #num: {}", size_,
            obs_frame_container_.front().mutable_obs_polygon_tree()->Length());
}

std::pair<math::AD2, double> ObsMap::Odom2Veh(
    const global::common::Pose &utm_pose, const global::common::Pose &odom_pose,
    const math::AD2 &pt, double heading) {
  double odom_yaw = common::GetYawFromPose(odom_pose);
  double dx = pt[0] - odom_pose.position().x();
  double dy = pt[1] - odom_pose.position().y();
  double sin_ = std::sin(odom_yaw);
  double cos_ = std::cos(odom_yaw);
  return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - odom_yaw};
}

std::pair<math::AD2, double> ObsMap::Veh2Odom(
    const global::common::Pose &utm_pose, const global::common::Pose &odom_pose,
    const math::AD2 &pt, double heading) {
  double odom_yaw = common::GetYawFromPose(odom_pose);
  double dx = pt[0];
  double dy = pt[1];
  double sin_ = std::sin(odom_yaw);
  double cos_ = std::cos(odom_yaw);
  return {{-dy * sin_ + dx * cos_ + odom_pose.position().x(),
           dx * sin_ + dy * cos_ + odom_pose.position().y()},
          heading + odom_yaw};
}

std::pair<math::AD2, double> ObsMap::Utm2Veh(
    const global::common::Pose &utm_pose, const global::common::Pose &odom_pose,
    const math::AD2 &pt, double heading) {
  double utm_yaw = common::GetYawFromPose(utm_pose);
  double dx = pt[0] - utm_pose.position().x();
  double dy = pt[1] - utm_pose.position().y();
  double sin_ = std::sin(utm_yaw);
  double cos_ = std::cos(utm_yaw);
  return {{dy * sin_ + dx * cos_, -dx * sin_ + dy * cos_}, heading - utm_yaw};
}

std::pair<math::AD2, double> ObsMap::Veh2Utm(
    const global::common::Pose &utm_pose, const global::common::Pose &odom_pose,
    const math::AD2 &pt, double heading) {
  double utm_yaw = common::GetYawFromPose(utm_pose);
  double dx = pt[0];
  double dy = pt[1];
  double sin_ = std::sin(utm_yaw);
  double cos_ = std::cos(utm_yaw);
  return {{-dy * sin_ + dx * cos_ + utm_pose.position().x(),
           dx * sin_ + dy * cos_ + utm_pose.position().y()},
          heading + utm_yaw};
}

std::pair<math::AD2, double> ObsMap::Utm2Odom(
    const global::common::Pose &utm_pose, const global::common::Pose &odom_pose,
    const math::AD2 &pt, double heading) {
  auto tmp = Utm2Veh(utm_pose, odom_pose, pt, heading);
  return Veh2Odom(utm_pose, odom_pose, tmp.first, tmp.second);
}

std::pair<math::AD2, double> ObsMap::Odom2Utm(
    const global::common::Pose &odom_pose, const global::common::Pose &utm_pose,
    const math::AD2 &pt, double heading) {
  auto tmp = Odom2Veh(utm_pose, odom_pose, pt, heading);
  return Veh2Utm(utm_pose, odom_pose, tmp.first, tmp.second);
}

std::pair<math::AD2, double> ObsMap::Mapping(const math::AD2 &pt,
                                             double heading,
                                             std::string &src2dst) {
  auto &odom_pose =
      DataCenter::Instance()->pose_base_link_in_odometry_msg.ptr->pose();
  auto &utm_pose =
      DataCenter::Instance()->pose_base_link_in_utm_msg.ptr->pose();
  if (auto func_ptr = coordinate_transformations_.find(src2dst);
      func_ptr != coordinate_transformations_.end()) {
    return func_ptr->second(this, utm_pose, odom_pose, pt, heading);
  }
}

void ObsMap::Reset() {
  obs_frame_container_.clear();
  size_ = 0;
}

void ObsMap::Push(ObsFrameContainer &frame) {
  time_++;
  size_ = size_ >= max_size_ ? max_size_ : size_ + 1;
  obs_frame_container_.push_front(frame);
}

void ObsMap::Push() {
  size_ = size_ >= max_size_ ? max_size_ : size_ + 1;
  obs_frame_container_.emplace_front(time_++);
}

}  // namespace obsmap
}  // namespace planning
}  // namespace neodrive
