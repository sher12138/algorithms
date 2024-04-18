#include "y_junction_law.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

YJunctionLaw::YJunctionLaw() : TrafficLaw("YJunctionLaw") {}

ErrorCode YJunctionLaw::Apply(TaskInfo& task_info,
                              const InsidePlannerData& inside_data,
                              const OutsidePlannerData& outside_data,
                              const Boundary& adc_boundary,
                              DecisionData* const decision_data,
                              TrafficLawContext* const traffic_law_context) {
  LOG_INFO("YJunctionLaw started");
  if (decision_data == nullptr || traffic_law_context == nullptr) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  const auto& plan_config = config::PlanningConfig::Instance()->plan_config();
  static constexpr double kIntervalThreshold = 1.0;
  DataCenter::Instance()->mutable_master_info()->set_stop_due_to_y_junction(
      false);
  for (const auto& [junction_ptr, overlap] :
       task_info.reference_line()->junctions()) {
    if (junction_ptr == nullptr) continue;
    if (static_cast<uint32_t>(
            autobot::cyberverse::Junction::JunctionType::Y_CROSS_ROAD) ==
            junction_ptr->Type() &&
        overlap.start_s + kIntervalThreshold > task_info.curr_sl().s() &&
        overlap.start_s <
            task_info.curr_sl().s() +
                plan_config.traffic_law.stop_signal_preview_distance) {
      // ask for take over and stop front stop signal.
      DataCenter::Instance()->mutable_master_info()->set_is_ask_for_takeover(
          true);
      DataCenter::Instance()->mutable_master_info()->set_stop_due_to_y_junction(
          true);
      int32_t obstacle_id = 0;
      auto ret = task_info.decision_data()->create_virtual_obstacle(
          overlap.start_s - plan_config.traffic_law.y_junction_stop_interval,
          VirtualObstacle::Y_JUNCTION, &obstacle_id);
      break;
    }
  }

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive