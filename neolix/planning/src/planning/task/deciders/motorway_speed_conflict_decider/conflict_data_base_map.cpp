#include "conflict_data_base_map.h"

namespace neodrive {
namespace planning {

ConflictDataBaseMap::ConflictDataBaseMap()
    : ConflictDataInterface("ConflictDataBaseOnPath") {}

std::vector<ConnectionConflictInfo>
ConflictDataBaseMap::ComputeConflictMeetingData(TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> meeting_data{};
  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return meeting_data;
  }
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  uint64_t ego_lane_id = traffic_conflict_zone_context.current_lane.id;
  Vec2d utm_ego_center = traffic_conflict_zone_context.ego_pos_utm;
  for (int i = 0; i < 2; ++i) {
    double extend_lane_length = (i == 0)
                                    ? 0.0
                                    : traffic_conflict_zone_context.current_lane
                                          .lane_ptr->TotalLength();
    if (traffic_conflict_zone_context.meeting_geoinfo[i]) {
      auto mg = traffic_conflict_zone_context.meeting_geoinfo[i];
      for (auto it = mg->begin(); it != mg->end(); it++) {
        auto& mlc = it->first;
        // ego
        double ego_rd{0.0}, ego_s{0.0};
        if (!PlanningMap::Instance()->GetResDistanceInLane(
                mlc, ego_lane_id, ego_rd, utm_ego_center)) {
          LOG_ERROR("Calculate Ego RD Failed!");
          return meeting_data;
        }
        ego_rd += extend_lane_length;
        ego_s = ego_rd + mlc.ego_s[1] - mlc.ego_s[0];
        // dynamic obs
        for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
          if (!interactive_agent_ids_.count(obs->id())) {
            continue;
          }
          if (obs->matched_lane_id() == mlc.lane_id) {
            ConnectionConflictInfo c;
            c.lane_id = mlc.lane_id;
            double agent_rd;
            Vec2d agent_center = obs->center();
            common::math::Vec2d p_agent(agent_center.x(), agent_center.y());
            // todo?
            if (!PlanningMap::Instance()->GetRSInLane(mlc, agent_rd, p_agent)) {
              LOG_ERROR("Calculate Agent RD Failed!");
              return meeting_data;
            }
            double obs_length = obs->length();
            // special case deal
            double meeting_s =
                std::min(std::abs(mlc.merging_s[1] - mlc.merging_s[0]), 10.0);
            std::vector<double> cab{agent_rd, agent_rd + meeting_s + obs_length,
                                    ego_s + 1.0, ego_rd};
            c.conflict_area_bound = cab;
            CalAgentInfo(obs, &c.agent);
            c.agent.id = obs->id();
            c.way_right = RightOfWay::UNKNOWN;
            SetConflictParam(&c);
            meeting_data.emplace_back(c);
          }
        }
      }
    }
  }
  LOG_INFO("Meeting Agents' size is {} !", meeting_data.size());

  return meeting_data;
}

std::vector<ConnectionConflictInfo>
ConflictDataBaseMap::ComputeConflictMergeInData(TaskInfo& task_info) {
  std::vector<ConnectionConflictInfo> merge_in_data{};
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  if (!HasMergingArea(task_info)) {
    LOG_INFO("No Merging Area!");
    return merge_in_data;
  }

  if (interactive_agent_ids_.empty()) {
    LOG_INFO("No Dynamic Obs!");
    return merge_in_data;
  }

  auto CalcRightOfWay =
      [](const uint64_t obs_match_lane_id,
         const TrafficConflictZoneContext& tczc) -> RightOfWay {
    bool agent_main = true;
    const auto& merging_ids = tczc.merging_ids;
    for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
      if (iter->id == obs_match_lane_id) {
        if (iter->orientation != ConflictLaneContext::mTurnType::Straight) {
          agent_main = false;
        }
      }
    }
    bool ego_main = false;
    const auto& current_lane = tczc.current_lane;
    if (current_lane.orientation == ConflictLaneContext::mTurnType::Straight) {
      ego_main = true;
    }
    if (ego_main && agent_main) {
      return RightOfWay::EQUAL;
    } else if (!ego_main && !agent_main) {
      return RightOfWay::EQUAL;
    } else if (ego_main && !agent_main) {
      LOG_INFO("The road of ego is main!");
      return RightOfWay::ADVANTAGE;
    } else if (!ego_main && agent_main) {
      return RightOfWay::UNADVANTAGE;
    }
    return RightOfWay::UNKNOWN;
  };

  const auto& multi_cipv_dynamic_obstacles_decision =
      task_info.current_frame()
          ->outside_planner_data()
          .motorway_speed_obstacle_context
          .multi_cipv_dynamic_obstacles_decision;
  for (int i = 0; i < 2; ++i) {
    if (traffic_conflict_zone_context.merging_geoinfo[i]) {
      for (const auto& msl :
           *(traffic_conflict_zone_context.merging_geoinfo[i])) {
        // dynamic obs
        for (const auto& obs : task_info.decision_data()->dynamic_obstacle()) {
          if (!interactive_agent_ids_.count(obs->id())) {
            continue;
          }
          if (obs->matched_lane_id() == msl.lane_id) {
            ConnectionConflictInfo c;
            c.lane_id = msl.lane_id;
            double agent_rd, ego_rd = msl.ego_rd;
            Vec2d agent_center = obs->center();
            common::math::Vec2d p_agent(agent_center.x(), agent_center.y());
            if (!PlanningMap::Instance()->GetRSInLane(msl, agent_rd, p_agent)) {
              LOG_ERROR("Calculate Agent RD Failed!");
            }
            // todo?: agent_rd < 0 -> 0; ego_rd < 0 -> 0.
            double obs_length = obs->length();
            // special case deal
            double merging_s = std::min(msl.merging_s, 10.0),
                   ego_s = std::min(msl.ego_s, 10.0);
            std::vector<double> cab{agent_rd, agent_rd + merging_s + obs_length,
                                    ego_rd + ego_s + 1.0, ego_rd};
            c.conflict_area_bound = cab;
            CalAgentInfo(obs, &c.agent);
            c.agent.id = obs->id();
            c.way_right = CalcRightOfWay(obs->matched_lane_id(),
                                         traffic_conflict_zone_context);
            SetConflictParam(&c);
            merge_in_data.emplace_back(c);
          }
        }
      }
    }
  }
  return merge_in_data;
}

}  // namespace planning
}  // namespace neodrive
