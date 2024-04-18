#include "conflict_info_for_straight.h"

#include "planning.pb.h"
#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

using MotorwayIntersectionStageState =
    neodrive::global::planning::MotorwayIntersectionStageState;

ConflictInfoForStraight::ConflictInfoForStraight()
    : ConflictInfoInterface("ConflictInfoForStraight") {}

std::vector<ConnectionConflictInfo>
ConflictInfoForStraight::ComputeConflictInfo(TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .motorway_speed_confilict_decider_config.straight_scenario_config
           .straight_conflict_enable) {
    LOG_INFO("Intersection straight scene not opened!");
    return {};
  }

  /// scenario and stage filter
  if (DataCenter::Instance()->master_info().curr_scenario() !=
      ScenarioState::MOTORWAY_INTERSECTION) {
    LOG_INFO("Current scenerio is not intersection!");
    return {};
  }
  const auto& curr_stage = DataCenter::Instance()
                               ->master_info()
                               .motorway_intersection_context()
                               .stage;
  if (curr_stage != MotorwayIntersectionStageState::STRAIGHT) {
    LOG_INFO("Current scenerio is not intersection of straight!");
    return {};
  }

  /// obstacle filter
  ConflictDataBasePrediction cdbpr;
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  const auto& inside_data = task_info.current_frame()->inside_planner_data();
  merge_in_agent_ids_.clear();
  CalcMergeInAgentsId(dynamic_obstacle, inside_data, task_info.curr_sl().s());
  cdbpr.set_interactive_agent_ids(
      merge_in_agent_ids_, neodrive::planning::MotorwayInteractiveType::NONE);
  auto ans = cdbpr.ComputeConflictMergeInData(task_info);

  meeting_agent_ids_.clear();
  CalcMeetingAgentsID(dynamic_obstacle, inside_data);
  cdbpr.set_interactive_agent_ids(
      meeting_agent_ids_, neodrive::planning::MotorwayInteractiveType::NONE);
  auto ans_meeting = cdbpr.ComputeConflictMeetingData(task_info);
  ans.insert(ans.end(), ans_meeting.begin(), ans_meeting.end());
  return ans;
}

void ConflictInfoForStraight::CalcMergeInAgentsId(
    const std::vector<Obstacle*>& dynamic_obs,
    const InsidePlannerData& inside_data, const double cur_s) {
  if (dynamic_obs.empty()) {
    return;
  }
  for (const auto& iter : dynamic_obs) {
    if (iter->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO(
          "Intersection's left turn merge: obs's type is pedestrian, id is {}!",
          iter->id());
      continue;
    }
    // double heading_diff =
    //     normalize_angle(iter->velocity_heading() - inside_data.vel_heading);
    // // U turn have risk, need specific tag
    // if (std::abs(heading_diff) > M_PI_2) {
    //   continue;
    // }
    // const auto& obs_boundary = iter->PolygonBoundary();
    // if (obs_boundary.start_s() >
    //     cur_s + VehicleParam::Instance()->length() -
    //         VehicleParam::Instance()->back_edge_to_center()) {
    //   continue;
    // }
    LOG_INFO("Intesection's straight merge in: add obs, id is {}!", iter->id());
    merge_in_agent_ids_.insert(iter->id());
  }
}

void ConflictInfoForStraight::CalcMeetingAgentsID(
    const std::vector<Obstacle*>& dynamic_obs,
    const InsidePlannerData& inside_data) {
  if (dynamic_obs.empty()) {
    return;
  }
  for (const auto& iter : dynamic_obs) {
    if (iter->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO(
          "Intesection's straight meeting: obs's type is pedestrian, id is {}!",
          iter->id());
      continue;
    }
    LOG_INFO("Intesection's straight meeting: add obs, id is {}!", iter->id());
    meeting_agent_ids_.insert(iter->id());
  }
}

}  // namespace planning
}  // namespace neodrive
