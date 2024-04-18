#pragma once

#include "environment.h"
#include "inside_planner_data.h"
#include "outside_planner_data.h"
#include "planning_data.h"
#include "src/planning/traffic_law/traffic_law_context.h"

namespace neodrive {
namespace planning {

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);
  ~Frame() = default;

  void set_planning_data(std::unique_ptr<PlanningData> &planning_data);
  const PlanningData &planning_data() const;
  PlanningData *mutable_planning_data();
  std::unique_ptr<PlanningData> planning_data_ptr();
  bool IsPlanningDataSafe() const;

 public:
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(uint32_t, sequence_num)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(TrafficLawContext, traffic_law_context)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(InsidePlannerData, inside_planner_data)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(OutsidePlannerData, outside_planner_data)

 private:
  uint32_t sequence_num_{0};
  std::unique_ptr<PlanningData> planning_data_{nullptr};
  TrafficLawContext traffic_law_context_{};
  InsidePlannerData inside_planner_data_{};
  OutsidePlannerData outside_planner_data_{};
};

}  // namespace planning
}  // namespace neodrive
