#include "decision.h"

namespace neodrive {
namespace planning {

Decision::Decision(const double& buffer, const DecisionType type)
    : buffer_(buffer), decision_type_(type) {}

Json::Value Decision::to_json() const {
  Json::Value root;
  std::string type = "UNKNOWN";
  switch (decision_type_) {
    case DecisionType::GO_LEFT:
      type = "GO_LEFT";
      break;
    case DecisionType::GO_RIGHT:
      type = "GO_RIGHT";
      break;
    case DecisionType::GO_UP:
      type = "GO_UP";
      break;
    case DecisionType::FOLLOW_DOWN:
      type = "FOLLOW_DOWN";
      break;
    case DecisionType::STOP_DOWN:
      type = "STOP_DOWN";
      break;
    case DecisionType::YIELD_DOWN:
      type = "YIELD_DOWN";
      break;
    case DecisionType::IGNORE:
      type = "IGNORE";
      break;
    default:
      break;
  }
  root["decision_type"] = type;
  root["buffer"] = buffer_;
  root["traj_index"] = traj_index_;
  root["nudge_distance"] = nudge_distance_;
  return root;
}

double Decision::buffer() const { return buffer_; }

const Decision::DecisionType& Decision::decision_type() const {
  return decision_type_;
}
int Decision::traj_index() const { return traj_index_; }

double Decision::nudge_distance() const { return nudge_distance_; }

double Decision::bounding_box_distance() const {
  return bounding_box_distance_;
}

int Decision::trajectory_point_index() const { return trajectory_point_index_; }

int Decision::ego_point_index() const { return ego_point_index_; }

void Decision::set_buffer(const double buf) { buffer_ = buf; }

void Decision::set_decision_type(const DecisionType& decision_type) {
  decision_type_ = decision_type;
}

void Decision::set_traj_index(const int index) { traj_index_ = index; }

void Decision::set_nudge_distance(const double nudge_distance) {
  nudge_distance_ = nudge_distance;
}

void Decision::set_bounding_box_distance(const double bounding_box_distance) {
  bounding_box_distance_ = bounding_box_distance;
}

void Decision::set_trajectory_point_index(const int trajectory_point_index) {
  trajectory_point_index_ = trajectory_point_index;
}

void Decision::set_ego_point_index(const int ego_point_index) {
  ego_point_index_ = ego_point_index;
}

}  // namespace planning
}  // namespace neodrive
