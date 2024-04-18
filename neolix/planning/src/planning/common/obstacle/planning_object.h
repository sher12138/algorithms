#pragma once

#include <vector>

#include "src/planning/common/decision/decision.h"
#include "common/math/polygon2d.h"

namespace neodrive {
namespace planning {

class PlanningObject {
 public:
  PlanningObject() = default;
  virtual ~PlanningObject() = default;

  const Polygon2d& polygon() const;
  const Polygon2d& polygon_with_shadow() const;
  Polygon2d* mutable_polygon();
  Polygon2d* mutable_polygon_with_shadow();

  const std::vector<Decision>& decision() const;
  const std::vector<Decision>& history_decision() const;
  std::vector<Decision>* mutable_decision();
  std::vector<Decision>* mutable_history_decision();

  void switch_to_history();
  bool has_decision_type(const Decision::DecisionType& decision_type) const;

  void setPolygon(const Polygon2d& polygon);

  virtual Json::Value to_json() const;

 private:
  Polygon2d polygon_;
  Polygon2d polygon_with_shadow_;

  std::vector<Decision> decision_;
  std::vector<Decision> history_decision_;
};

}  // namespace planning
}  // namespace neodrive
