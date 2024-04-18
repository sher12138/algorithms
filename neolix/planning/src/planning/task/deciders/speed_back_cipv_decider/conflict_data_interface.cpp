#include "conflict_data_interface.h"

namespace neodrive {
namespace planning {

const bool BackDataInterface::HasMergingArea(TaskInfo& task_info) const {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  bool has_merge_area = false;
  if (!((traffic_conflict_zone_context.type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_INFO("Current Type is Merge Area!");
    has_merge_area = true;
  }
  if (!((traffic_conflict_zone_context.near_type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.near_type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_INFO("Near Type is Merge Area!");
    has_merge_area = true;
  }
  if (!((traffic_conflict_zone_context.rear_type ==
         TrafficConflictZoneContext::connectionType::Merging) ||
        (traffic_conflict_zone_context.rear_type ==
         TrafficConflictZoneContext::connectionType::NMerging))) {
    LOG_INFO("Rear Type is Merge Area!");
    has_merge_area = true;
  }
  return has_merge_area;
}

const bool BackDataInterface::CalAgentInfo(const Obstacle* obs,
                                           VehicleInfo* agent_info) const {
  Vec2d xy_p(obs->center().x(), obs->center().y());
  Vec2d sl_p(obs->center_sl().s(), obs->center_sl().l());
  Vec2d sl_v(obs->speed(), 0.0);
  Vec2d sl_a(0.0, 0.0);
  double width = obs->width();
  double length = obs->length();
  std::vector<double> suspension{0.5 * length, 0.5 * length, 0.5 * width,
                                 0.5 * width};
  std::vector<double> param_limit{2.0, 1.0, 5.5};
  *agent_info =
      VehicleInfo(obs->id(), xy_p, sl_p, sl_v, sl_a, obs->velocity_heading(),
                  suspension, 0.05, param_limit);
  agent_info->heading_diff = obs->matched_lane_heading_deviation();
  agent_info->obs_offset = obs->matched_lane_offset();
  return true;
}

const bool BackDataInterface::SetConflictParam(
    ConnectionConflictInfo* c) const {
  c->back_type = scenario_type_;
  c->way_right = way_right_;
  auto& param = c->param;
  auto set_param = [](const auto& config, const int w,
                      GameTheoryParam& p) -> void {
    p.param_a = config.param_a[w];
    p.param_c = config.param_c[w];
    p.param_margin = config.param_margin[w];
    p.rush_time = config.rush_time[w];
    p.yield_time = config.yield_time[w];
    p.yield_advance_response_distance = config.yield_advance_response_distance;
    p.rush_margin_distance = config.rush_margin_distance;
  };

  int e_way = 0;
  switch (way_right_) {
    case RightOfWay::UNKNOWN:
      e_way = 0;
      break;
    case RightOfWay::ADVANTAGE:
      e_way = 1;
      break;
    case RightOfWay::UNADVANTAGE:
      e_way = 2;
      break;
    case RightOfWay::EQUAL:
      e_way = 3;
      break;
    default:
      e_way = 0;
      break;
  }

  switch (scenario_type_) {
    case BackInteractiveType::MERGE:
      set_param(back_config_.merge_in_config.game_params, e_way, param);
      break;
    case BackInteractiveType::MEETING:
      set_param(back_config_.meeting_config.game_params, e_way, param);
      break;
    case BackInteractiveType::CUSTOM:
      set_param(back_config_.custom_scenario_config.game_params, e_way, param);
      break;
    default:
      break;
  }

  return true;
}

}  // namespace planning
}  // namespace neodrive
