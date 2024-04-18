#pragma once

#include "traffic_law.h"
#include "traffic_law_context.h"

namespace neodrive {
namespace planning {

class StopSignLaw : public TrafficLaw {
 public:
  StopSignLaw();
  virtual ~StopSignLaw() = default;

  virtual ErrorCode Apply(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
      DecisionData* const decision_data,
      TrafficLawContext* const traffic_law_context) override;

 private:
  ErrorCode NormalCruiseManager(const ReferenceLinePtr& refer_line,
                                const Boundary& adc_boundary);

  ErrorCode PreStopManager(const TaskInfo& task_info,
                           const Boundary& adc_boundary);

  ErrorCode StopManager();

  // ErrorCode creep_manager(const Boundary& adc_boundary);

  ErrorCode FinishManager();

 private:
  //   void update_stop_sign_context(const TaskInfo& task_info,
  //                                 const Boundary& adc_boundary,
  //                                 TrafficLawContext* traffic_law_context);

  void FindFrontStopSign(const ReferenceLinePtr& task_info,
                         const Boundary& adc_boundary,
                         std::vector<RouteStopSign>* stop_sign);

  void UpdateStopSignState(const double& vel, const Boundary& adc_boundary,
                           RouteStopSign* stop_sign);

  bool IsNearStopSign(const Boundary& adc_boundary,
                      const RouteStopSign& stop_sign);

 private:
  RouteStopSign current_stop_sign_;
  StopSignState stop_sign_state_;
  double last_stop_time_;
  double stop_line_s_;
  bool need_stop_ = false;
};

}  // namespace planning
}  // namespace neodrive
