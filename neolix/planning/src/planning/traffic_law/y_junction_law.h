#pragma once

#include "traffic_law.h"

namespace neodrive {
namespace planning {

class YJunctionLaw : public TrafficLaw {
 public:
  YJunctionLaw();
  virtual ~YJunctionLaw() = default;

  virtual ErrorCode Apply(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
      DecisionData* const decision_data,
      TrafficLawContext* const traffic_law_context) override;
};

}  // namespace planning
}  // namespace neodrive