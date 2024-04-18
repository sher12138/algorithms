/**
 * @file agent.h
 * data struct define
 **/

#pragma once
#include <json/json.h>
#include "common_geometry.pb.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "planning_map/planning_map.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {

using neodrive::global::perception::PerceptionObstacle_Type;
using neodrive::global::perception::PerceptionObstacles;
using PerceptionObstaclesShrPtr = std::shared_ptr<PerceptionObstacles>;
using neodrive::global::common::Point3D;
using neodrive::global::perception::PerceptionObstacle;

class Agent {
 public:
  Agent() = default;
  ~Agent() = default;
  Agent(const PerceptionObstacle& perception_obstacle);
  Json::Value to_json() const;
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, measurement_time);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(PlanningRLMap::MapPoint, position);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<double>, extent);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Point3D, velocity);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(size_t, track_id);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(PerceptionObstacle_Type, type);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<double>,
                                       label_probabilities);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::string, type_string);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, validate);

 private:
  double measurement_time_;
  PlanningRLMap::MapPoint position_;
  std::vector<double> extent_;
  Point3D velocity_;

  size_t track_id_;
  PerceptionObstacle_Type type_;
  std::vector<double> label_probabilities_{0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> type_string_list_{
      "UNKNOWN",    "UNKNOWN_MOVABLE", "UNKNOWN_UNMOVABLE",
      "PEDESTRIAN", "BICYCLE",         "VEHICLE"};
  std::string type_string_{type_string_list_[0]};
  bool validate_ = false;
};
// alias for interface using
using Agent2d = std::vector<Agent>;

class Agents {
 public:
  Agents() = default;
  ~Agents() = default;
  Agents(const PerceptionObstaclesShrPtr& perception_obstacles, int mode);
  Json::Value to_json() const;
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(Agent2d, agents);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, validate);

 private:
  Agent2d agents_;
  bool validate_ = true;
};

}  // namespace planning_rl
}  // namespace neodrive
