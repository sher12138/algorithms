#include "conflict_info_for_merge_in.h"
namespace neodrive {
namespace planning {

BackInfoForMergeIn::BackInfoForMergeIn()
    : BackInfoInterface("BackInfoForMergeIn") {}

std::vector<ConnectionConflictInfo> BackInfoForMergeIn::ComputeConflictInfo(
    TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .back_speed_confilict_decider_config.merge_in_config
           .merge_in_conflict_enable) {
    return {};
  }

  /// obstacle filter
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  BackDataBasePath cdbp;
  // for merge in scenario
  merge_in_agent_ids_.clear();
  const auto& dp_st_ignore_dynamic_obs =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->speed_obstacle_context.dp_st_map_ignore_dynamic_obs_id;
  for (const auto& iter : dynamic_obstacle) {
    if (iter->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("left turn and merge: obs's type is pedestrian, id is {}!",
               iter->id());
      continue;
    }
    if (dp_st_ignore_dynamic_obs.find(iter->id()) !=
        dp_st_ignore_dynamic_obs.end()) {
      LOG_INFO("dp st ignore dynamic obs id is {}", iter->id());
      continue;
    }
    double heading_diff = normalize_angle(
        iter->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    if (std::abs(heading_diff) > M_PI_2) {
      continue;
    }
    const auto& obs_boundary = iter->PolygonBoundary();
    if (obs_boundary.start_s() >
        task_info.curr_sl().s() + VehicleParam::Instance()->length() -
            VehicleParam::Instance()->back_edge_to_center()) {
      continue;
    }
    merge_in_agent_ids_.insert(iter->id());
  }
  cdbp.set_interactive_agent_ids(
      merge_in_agent_ids_, neodrive::planning::BackInteractiveType::MERGE);
  auto ans_mergein = cdbp.ComputeConflictMergeInData(task_info);

  std::sort(
      ans_mergein.begin(), ans_mergein.end(),
      [](const ConnectionConflictInfo& a, const ConnectionConflictInfo& b) {
        return a.conflict_area_bound[3] < b.conflict_area_bound[3];
      });
  return ans_mergein;
}

}  // namespace planning
}  // namespace neodrive
