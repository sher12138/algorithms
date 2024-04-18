#include "conflict_info_for_left_turn.h"

#include "planning.pb.h"
#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

using MotorwayIntersectionStageState =
    neodrive::global::planning::MotorwayIntersectionStageState;

ConflictInfoForLeftTurn::ConflictInfoForLeftTurn()
    : ConflictInfoInterface("ConflictInfoForLeftTurn") {}

std::vector<ConnectionConflictInfo>
ConflictInfoForLeftTurn::ComputeConflictInfo(TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .motorway_speed_confilict_decider_config.turn_left_scenario_config
           .turn_left_conflict_enable) {
    return {};
  }

  /// scenario and stage filter
  const auto& curr_scenario =
      DataCenter::Instance()->master_info().curr_scenario();
  if (curr_scenario != ScenarioState::MOTORWAY_INTERSECTION) {
    return {};
  }
  const auto& curr_stage = DataCenter::Instance()
                               ->master_info()
                               .motorway_intersection_context()
                               .stage;
  if (!(curr_stage == MotorwayIntersectionStageState::LEFT_WAIT_ZONE ||
        curr_stage == MotorwayIntersectionStageState::UNPROTECTED_TURN_LEFT ||
        curr_stage == MotorwayIntersectionStageState::PROTECTED_TURN_LEFT)) {
    return {};
  }

  /// obstacle filter
  ConflictDataBasePath cdbp;
  // for meeting in scenario
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  meeting_agent_ids_.clear();
  for (const auto& iter : dynamic_obstacle) {
    if (iter->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("left turn and meeting: obs's type is pedestrian, id is {}!",
               iter->id());
      continue;
    }
    meeting_agent_ids_.insert(iter->id());
  }
  cdbp.set_interactive_agent_ids(
      meeting_agent_ids_,
      neodrive::planning::MotorwayInteractiveType::LEFT_MEETING);
  auto ans = cdbp.ComputeConflictMeetingData(task_info);

  // for merge in scenario
  merge_in_agent_ids_.clear();
  for (const auto& iter : dynamic_obstacle) {
    if (iter->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("left turn and merge: obs's type is pedestrian, id is {}!",
               iter->id());
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
            VehicleParam::Instance()->back_edge_to_center() + 2.0) {
      continue;
    }
    merge_in_agent_ids_.insert(iter->id());
  }
  cdbp.set_interactive_agent_ids(
      merge_in_agent_ids_,
      neodrive::planning::MotorwayInteractiveType::LEFT_MERGE);
  auto ans_mergein = cdbp.ComputeConflictMergeInData(task_info);

  ans.insert(ans.end(), ans_mergein.begin(), ans_mergein.end());
  std::sort(
      ans.begin(), ans.end(),
      [](const ConnectionConflictInfo& a, const ConnectionConflictInfo& b) {
        return a.conflict_area_bound[3] < b.conflict_area_bound[3];
      });
  return ans;
}

}  // namespace planning
}  // namespace neodrive
