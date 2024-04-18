#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/config/planning_config.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/math/public_math/utility.h"
#include "traffic_law.h"

namespace neodrive {
namespace planning {

class TrafficLightLaw : public TrafficLaw {
 public:
  TrafficLightLaw();
  virtual ~TrafficLightLaw() = default;
  virtual ErrorCode Apply(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
      DecisionData* const decision_data,
      TrafficLawContext* const traffic_law_context) override;

 private:
  void ComputeSignalOverlapsInfo(const TaskInfo& task_info,
                                 const Boundary& adc_boundary,
                                 TrafficLightTable* const front_traffic_lights,
                                 TrafficLawContext* const traffic_law_context);

  bool CombinePerceptionTrafficLight(const TaskInfo& task_info,
                                     TrafficLightTable& traffic_light_table);

  void TrafficLightDecider(const TaskInfo& task_info,
                           const Boundary& adc_boundary,
                           TrafficLightTable& traffic_light_table,
                           TrafficLawContext* const traffic_law_context);

  const config::AutoPlanConfig::TrafficLaw& traffic_law_config_ =
      config::PlanningConfig::Instance()->plan_config().traffic_law;
  bool is_motorway_{false};

  double distance_to_selected_end_s_{0.0};
};

}  // namespace planning
}  // namespace neodrive
