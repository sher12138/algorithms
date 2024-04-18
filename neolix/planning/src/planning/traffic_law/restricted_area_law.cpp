#include "restricted_area_law.h"

#include "global_adc_status.pb.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

RestrictedAreaLaw::RestrictedAreaLaw() : TrafficLaw("RestrictedAreaLaw") {}

ErrorCode RestrictedAreaLaw::Apply(
    TaskInfo& task_info, const InsidePlannerData& inside_data,
    const OutsidePlannerData& outside_data, const Boundary& adc_boundary,
    DecisionData* const decision_data,
    TrafficLawContext* const traffic_law_context) {
  LOG_INFO("RestrictedAreaLaw started");
  DataCenter::Instance()->mutable_master_info()->set_restricted_area(false);
  if (decision_data == nullptr || traffic_law_context == nullptr) {
    LOG_INFO("decision_data or traffic_law_context is nullptr");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  for (const auto& [geofence_id, geofence_start_s, geofence_end_s] :
       task_info.reference_line()->geo_fence_overlaps()) {
    LOG_INFO(
        "current s = {:.4f}, geofence_start_s = {:.4f}, geofence_end_s = "
        "{:.4f}",
        task_info.curr_sl().s(), geofence_start_s, geofence_end_s);
    if (geofence_start_s - 50.0 > task_info.curr_sl().s()) {
      LOG_INFO("far away from restricted area.");
      break;
    } else if (task_info.curr_sl().s() < geofence_end_s) {
      // ask for take over .
      LOG_INFO("Restricted area.");
      DataCenter::Instance()->mutable_master_info()->set_restricted_area(true);
      break;
    }
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive