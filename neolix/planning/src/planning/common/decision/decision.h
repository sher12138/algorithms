#pragma once

#include <string>

#include "src/planning/public/planning_lib_header.h"

namespace neodrive {
namespace planning {

class Decision {
 public:
  enum class DecisionType {
    GO_LEFT = 0,
    GO_RIGHT = 1,
    GO_UP = 2,
    FOLLOW_DOWN = 3,
    STOP_DOWN = 4,
    YIELD_DOWN = 5,
    IGNORE = 6,
    UNKNOWN = 7,
    PATH_ATTENTION = 8,
  };

 public:
  Decision() = default;
  ~Decision() = default;
  Decision(const double& buffer, const DecisionType type);

  Json::Value to_json() const;

  double buffer() const;
  const DecisionType& decision_type() const;
  int traj_index() const;
  double nudge_distance() const;
  double bounding_box_distance() const;
  int trajectory_point_index() const;
  int ego_point_index() const;

  void set_buffer(const double buf);
  void set_decision_type(const DecisionType& decision_type);
  void set_traj_index(const int index);
  void set_nudge_distance(const double nudge_distance);
  void set_bounding_box_distance(const double bounding_box_distance);
  void set_trajectory_point_index(const int traj_point_index);
  void set_ego_point_index(const int ego_point_index);

 private:
  double buffer_ = 0.0;
  DecisionType decision_type_ = DecisionType::UNKNOWN;
  int traj_index_ = -1;
  double nudge_distance_ = 100.0;
  double bounding_box_distance_ = -1;
  int trajectory_point_index_ = -1;
  int ego_point_index_ = -1;
};

}  // namespace planning
}  // namespace neodrive
