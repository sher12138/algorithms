/**
 * @file frame.cpp
 **/

#include "ego.h"

namespace neodrive {
namespace planning_rl {

Ego::Ego(const LocalizationEstimateShrPtr& localization) {
  try {
    auto pose = localization->pose();
    auto orientation = pose.orientation();
    double yaw = planning_rl::GetYawFromQuaternion(orientation);
    position_.x = pose.position().x();
    position_.y = pose.position().y();
    position_.z = pose.position().z();
    position_.heading = yaw;
    velocity_ = pose.linear_velocity();
    linear_acceleration_ = pose.linear_acceleration();
    measurement_time_ = localization->measurement_time();
    validate_ = true;
  } catch (const std::exception& e) {
    validate_ = false;
    std::cerr << e.what() << '\n';
  }
}

Ego::Ego(const PoseStampedShrPtr& odom_pose, const TwistStampedShrPtr& twist) {
  try {
    auto& pose = odom_pose->pose();
    auto& orientation = pose.orientation();
    double yaw = planning_rl::GetYawFromQuaternion(orientation);
    position_.x = pose.position().x();
    position_.y = pose.position().y();
    position_.z = pose.position().z();
    position_.heading = yaw;
    velocity_.set_x(twist->twist().linear().x());
    velocity_.set_y(twist->twist().linear().y());
    velocity_.set_z(twist->twist().linear().z());
    // velocity_ = twist->twist().linear();
    linear_acceleration_.set_x(twist->twist().linear_acc().x());
    linear_acceleration_.set_y(twist->twist().linear_acc().y());
    linear_acceleration_.set_z(twist->twist().linear_acc().z());
    // linear_acceleration_ = twist->twist().linear_acc();
    measurement_time_ = 0.0;
    validate_ = true;
  } catch (const std::exception& e) {
    validate_ = false;
    std::cerr << e.what() << '\n';
  }
}

Json::Value Ego::to_json() const {
  Json::Value root;
  Json::Value json_position;
  Json::Value json_velocity;
  Json::Value json_linear_acceleration;

  json_position["x"] = position_.x;
  json_position["y"] = position_.y;
  json_position["z"] = position_.z;
  json_position["heading"] = position_.heading;

  json_velocity["x"] = velocity_.x();
  json_velocity["y"] = velocity_.y();
  json_velocity["z"] = velocity_.z();

  json_linear_acceleration["x"] = linear_acceleration_.x();
  json_linear_acceleration["y"] = linear_acceleration_.y();
  json_linear_acceleration["z"] = linear_acceleration_.z();

  root["position"] = json_position;
  root["velocity"] = json_velocity;
  root["linear_acceleration"] = json_linear_acceleration;
  root["measurement_time"] = measurement_time_;
  root["validate"] = validate_;
  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
