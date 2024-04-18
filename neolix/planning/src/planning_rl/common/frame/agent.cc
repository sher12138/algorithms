/**
 * @file agent.cpp
 **/

#include "agent.h"

namespace neodrive {
namespace planning_rl {

Agent::Agent(const PerceptionObstacle& perception_obstacle) {
  try {
    measurement_time_ = perception_obstacle.timestamp();
    track_id_ = perception_obstacle.id();
    position_.x = perception_obstacle.position().x();
    position_.y = perception_obstacle.position().y();
    position_.z = perception_obstacle.position().z();
    position_.heading = perception_obstacle.theta();
    velocity_.set_x(perception_obstacle.velocity().x());
    velocity_.set_y(perception_obstacle.velocity().y());
    velocity_.set_z(perception_obstacle.velocity().z());
    extent_.push_back(perception_obstacle.length());
    extent_.push_back(perception_obstacle.width());
    extent_.push_back(perception_obstacle.height());
    type_ = perception_obstacle.type();
    type_string_ = type_string_list_[type_];
    label_probabilities_[type_] = 1.0;
    validate_ = true;
  } catch (const std::exception& e) {
    validate_ = false;
    std::cerr << e.what() << '\n';
  }
}

Json::Value Agent::to_json() const {
  Json::Value root;
  Json::Value json_position;
  Json::Value json_velocity;
  Json::Value json_extent;
  Json::Value json_label_probabilities;

  json_position["x"] = position_.x;
  json_position["y"] = position_.y;
  json_position["z"] = position_.z;
  json_position["heading"] = position_.heading;

  json_velocity["x"] = velocity_.x();
  json_velocity["y"] = velocity_.y();
  json_velocity["z"] = velocity_.z();

  json_extent["length"] = extent_[0];
  json_extent["width"] = extent_[1];
  json_extent["height"] = extent_[2];

  root["position"] = json_position;
  root["velocity"] = json_velocity;
  root["extent"] = json_extent;
  root["measurement_time"] = measurement_time_;
  root["validate"] = validate_;
  root["type"] = type_;
  for (size_t i = 0; i < label_probabilities_.size(); i++) {
    json_label_probabilities.append(label_probabilities_[i]);
  }
  root["label_probabilities"] = json_label_probabilities;
  return root;
}

Agents::Agents(const PerceptionObstaclesShrPtr& perception_obstacles,
               int mode) {
  agents_.clear();
  if (perception_obstacles->perception_obstacle_size() > 0) {
    if (mode == 1) {
      for (const auto& obstacle :
           perception_obstacles->odometry_perception_obstacle()) {
        Agent agent(obstacle);
        if (!agent.validate()) {
          validate_ = false;
        }
        agents_.push_back(Agent(obstacle));
      }
    } else {
      for (const auto& obstacle : perception_obstacles->perception_obstacle()) {
        Agent agent(obstacle);
        if (!agent.validate()) {
          validate_ = false;
        }
        agents_.push_back(Agent(obstacle));
      }
    }
  } else {
    // validate_ = false;
    return;
  }
}

Json::Value Agents::to_json() const {
  Json::Value root;
  Json::Value json_agents;
  for (std::size_t i = 0; i < agents_.size(); ++i) {
    json_agents.append(agents_[i].to_json());
  }
  root["validate"] = validate_;
  root["agent"] = json_agents;
  return root;
}

}  // namespace planning_rl
}  // namespace neodrive
