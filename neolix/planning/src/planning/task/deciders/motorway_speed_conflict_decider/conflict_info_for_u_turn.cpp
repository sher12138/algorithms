#include "conflict_info_for_u_turn.h"

#include "conflict_data_base_map.h"
namespace neodrive {
namespace planning {

ConflictInfoForUTurn::ConflictInfoForUTurn()
    : ConflictInfoInterface("ConflictInfoForUTurn") {}

std::vector<ConnectionConflictInfo> ConflictInfoForUTurn::ComputeConflictInfo(
    TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .motorway_speed_confilict_decider_config.u_turn_scenario_config
           .u_turn_conflict_enable) {
    return {};
  }

  ConflictDataBaseMap cdbp;
  // for meeting scenario
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  meeting_agent_ids_.clear();
  for (const auto& iter : dynamic_obstacle) {
    meeting_agent_ids_.insert(iter->id());
  }
  cdbp.set_interactive_agent_ids(
      meeting_agent_ids_, neodrive::planning::MotorwayInteractiveType::NONE);
  std::vector<ConnectionConflictInfo> ans =
      cdbp.ComputeConflictMeetingData(task_info);

  std::vector<ConnectionConflictInfo> ans_mergein =
      cdbp.ComputeConflictMergeInData(task_info);

  ans.insert(ans.end(), ans_mergein.begin(), ans_mergein.end());
  return ans;
}

}  // namespace planning
}  // namespace neodrive
