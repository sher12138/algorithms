#pragma once

#include "src/planning/common/boundary.h"
#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/task/task_info.h"
#include "traffic_law_context.h"

namespace neodrive {
namespace planning {

using ObstacleBoundaryMap = std::map<int, Boundary>;

class TrafficLaw {
 public:
  explicit TrafficLaw(const std::string& name) : name_(name) {}
  virtual ~TrafficLaw() = default;
  virtual ErrorCode Apply(TaskInfo& task_info,
                          const InsidePlannerData& inside_data,
                          const OutsidePlannerData& outside_data,
                          const Boundary& adc_boundary,
                          DecisionData* const decision_data,
                          TrafficLawContext* const traffic_law_context) = 0;
  virtual const std::string& Name() const { return name_; }

 private:
  std::string name_ = "";
};

}  // namespace planning
}  // namespace neodrive
