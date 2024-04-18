#pragma once

#include "traffic_law.h"

namespace neodrive {
namespace planning {

class CrosswalkLaw : public TrafficLaw {
 public:
  CrosswalkLaw();
  virtual ~CrosswalkLaw() = default;
  virtual ErrorCode Apply(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
      DecisionData* const decision_data,
      TrafficLawContext* const traffic_law_context) override;

 private:
  void FindFrontCrosswalk(const TaskInfo& task_info,
                          const Boundary& adc_boundary,
                          std::vector<CrosswalkSignal>& crosswalks);

  bool ComputeDecisionInfo(const TaskInfo& task_info,
                           const Boundary& adc_boundary,
                           DecisionData* const decision_data,
                           const CrosswalkSignal& curr_crosswalk,
                           CrosswalkDecisionContext* context);

 private:
  double last_stop_time_ = 0.0;
  uint64_t invalid_crosswalk_id_ = 0;
};

}  // namespace planning
}  // namespace neodrive
