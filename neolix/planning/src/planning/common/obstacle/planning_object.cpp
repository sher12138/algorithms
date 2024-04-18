#include "planning_object.h"

namespace neodrive {
namespace planning {

const Polygon2d& PlanningObject::polygon() const { return polygon_; }

const Polygon2d& PlanningObject::polygon_with_shadow() const {
  return polygon_with_shadow_;
}

Polygon2d* PlanningObject::mutable_polygon() { return &polygon_; }

Polygon2d* PlanningObject::mutable_polygon_with_shadow() {
  return &polygon_with_shadow_;
}

const std::vector<Decision>& PlanningObject::decision() const {
  return decision_;
}

const std::vector<Decision>& PlanningObject::history_decision() const {
  return history_decision_;
}

std::vector<Decision>* PlanningObject::mutable_decision() { return &decision_; }

std::vector<Decision>* PlanningObject::mutable_history_decision() {
  return &history_decision_;
}

void PlanningObject::switch_to_history() {
  history_decision_ = decision_;
  decision_.clear();
}

bool PlanningObject::has_decision_type(
    const Decision::DecisionType& decision_type) const {
  for (const auto& decision : decision_) {
    if (decision.decision_type() == decision_type) return true;
  }
  return false;
}

void PlanningObject::setPolygon(const Polygon2d& polygon) {
  polygon_ = polygon;
}

Json::Value PlanningObject::to_json() const {
  Json::Value root;
  for (std::size_t i = 0; i < decision_.size(); ++i) {
    root["decision"].append(decision_[i].to_json());
  }
  for (std::size_t i = 0; i < history_decision_.size(); ++i) {
    root["history_decision"].append(history_decision_[i].to_json());
  }
  for (std::size_t i = 0; i < polygon_.points().size(); ++i) {
    Json::Value point;
    point["x"] = polygon_.points()[i].x();
    point["y"] = polygon_.points()[i].y();
    root["polygon"].append(point);
  }
  return root;
}

}  // namespace planning
}  // namespace neodrive
