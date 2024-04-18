#include "conflict_info_for_merge_in.h"
namespace neodrive {
namespace planning {

namespace {

using JunctionType = autobot::cyberverse::Junction::JunctionType;

bool JudgeStraight(const std::vector<PathPoint>& points, const double length,
                   std::pair<double, bool>& straight_valid_length) {
  straight_valid_length.first = 0.0;
  for (auto& pt : points) {
    if (pt.kappa() < 0.01 && pt.kappa() > -0.01) {
      straight_valid_length.first = pt.s();
      if (pt.s() > length) return true;
    } else {
      if (pt.kappa() >= 0) {
        if (straight_valid_length.second) return false;
        straight_valid_length.second = false;
      } else {
        straight_valid_length.second = true;
      }
      return false;
    }
  }
  return false;
}
}  // namespace

ConflictInfoForMergeIn::ConflictInfoForMergeIn()
    : ConflictInfoInterface("ConflictInfoForMergeIn") {}

std::vector<ConnectionConflictInfo> ConflictInfoForMergeIn::ComputeConflictInfo(
    TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .motorway_speed_confilict_decider_config
           .merge_in_road_scenario_config.merge_in_conflict_enable) {
    return {};
  }
  std::vector<int> front_risk_obs{};
  std::vector<ConnectionConflictInfo> cibp_info =
      CalMergeConflictInfoBasePath(task_info, front_risk_obs);
  std::vector<ConnectionConflictInfo> cibm_info =
      CalMergeConflictInfoBaseMap(task_info, cibp_info, front_risk_obs);
  cibp_info.insert(cibp_info.end(), cibm_info.begin(), cibm_info.end());
  return cibp_info;
}

std::vector<ConnectionConflictInfo>
ConflictInfoForMergeIn::CalMergeConflictInfoBasePath(
    TaskInfo& task_info, std::vector<int>& front_risk_obs) {
  ConflictDataBasePath cdbp;
  adc_current_s_ = task_info.curr_sl().s();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
  adc_front_edge_s_ =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  const auto& merging_ids = traffic_conflict_zone_context.merging_ids;
  std::unordered_set<uint64_t> merging_lane_ids;

  if (!speed_planner_common::InMergingArea(task_info)) {
    LOG_INFO("ego not in merge , skip!");
    return std::vector<ConnectionConflictInfo>{};
  }
  if (!EgoScenarioIsCruise(task_info)) {
    LOG_INFO("ego not in cruise ,skip!");
    return std::vector<ConnectionConflictInfo>{};
  }

  for (auto iter = merging_ids.begin(); iter != merging_ids.end(); iter++) {
    merging_lane_ids.insert(iter->id);
    LOG_INFO("add match merging lane id:{}", iter->id);
  }
  const auto& adc_lane = traffic_conflict_zone_context.current_lane.id;
  LOG_INFO("adc lane id:{}", adc_lane);
  for (const auto& obs : dynamic_obstacle) {
    if (obs->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("cibp: obs's type is pedeatrian, id is {}!", obs->id());
      continue;
    }
    double heading_diff = normalize_angle(
        obs->velocity_heading() -
        task_info.current_frame()->inside_planner_data().vel_heading);
    // reverse
    if (std::abs(heading_diff) > M_PI_2) {
      LOG_INFO("cibp: obs [{}] heading diff {:.3f} > M_PI_2, ignore.",
               obs->id(), heading_diff);
      continue;
    }
    // behind adc and slow
    const auto& obs_boundary = obs->PolygonBoundary();
    if ((obs_boundary.end_s() < adc_current_s_) &&
        (obs->speed() <
         adc_current_v_ * config_->consider_adc_back_speed_ratio)) {
      LOG_INFO("cibp: obs [{}] behind adc and slow, ignore.", obs->id());
      continue;
    }
    // ahead adc
    if (obs_boundary.start_s() >
        adc_front_edge_s_ + config_->consider_adc_front_dis) {
      if (merging_lane_ids.find(obs->matched_lane_id()) !=
          merging_lane_ids.end()) {
        CalcRelativeLatDistance(task_info, obs, front_risk_obs);
      }
      LOG_INFO("cibp: obs [{}] ahead adc, ignore.", obs->id());
      continue;
    }
    LOG_INFO("obs [{}] lane id is {}.", obs->id(), obs->matched_lane_id());
    if (obs->matched_lane_id() == adc_lane) {
      if (JudgeSameLane(obs, task_info.reference_line(), adc_current_s_) &&
          FilterWhenInSameLane(obs, task_info)) {
        LOG_INFO("obs [{}] in same lane and satisfy ignore solution, ignore.",
                 obs->id());
        continue;
      }
    } else if (merging_lane_ids.find(obs->matched_lane_id()) !=
               merging_lane_ids.end()) {
      LOG_INFO("obs [{}] on merging lane.add to merge agents", obs->id());
      cibp_agents_.insert(obs->id());
      continue;
    } else {
      LOG_INFO("obs [{}] can't handling, ignore.", obs->id());
      continue;
    }
    LOG_INFO("skip obs {}", obs->id());
  }

  cdbp.set_interactive_agent_ids(
      cibp_agents_, neodrive::planning::MotorwayInteractiveType::MERGE_IN_ROAD);
  std::vector<ConnectionConflictInfo> ans_mergein =
      cdbp.ComputeConflictMergeInData(task_info);
  return ans_mergein;
}

std::vector<ConnectionConflictInfo>
ConflictInfoForMergeIn::CalMergeConflictInfoBaseMap(
    TaskInfo& task_info, const std::vector<ConnectionConflictInfo>& cibp_info,
    const std::vector<int>& front_risk_obs) {
  std::vector<ConnectionConflictInfo> cibm{};
  auto& conflict_decision = task_info.current_frame()
                                ->mutable_outside_planner_data()
                                ->traffic_conflict_decision;
  conflict_decision.Reset();
  double adc_current_v = task_info.current_frame()->inside_planner_data().vel_v;
  if (adc_current_v <= 1.0) {
    return cibm;
  }
  ConflictDataBaseMap cdbm;
  if (cibp_agents_.empty() && front_risk_obs.empty()) {
    return std::vector<ConnectionConflictInfo>{};
  }
  std::unordered_set<int> cibp_ids{};
  for (const auto& ans : cibp_info) {
    if (ans.conflict_area_bound[0] > 1e+7) {
      continue;
    }
    cibp_ids.insert(ans.agent.id);
  }
  for (const auto& id : cibp_agents_) {
    if (cibp_ids.find(id) == cibp_ids.end()) {
      LOG_INFO("cibm: add obs id: {}", id);
      cibm_agents_.insert(id);
    }
  }
  for (const auto& id : front_risk_obs) {
    LOG_INFO("cibm: add front risk obs id is {}!", id);
    cibm_agents_.insert(id);
  }
  cdbm.set_interactive_agent_ids(
      cibm_agents_, neodrive::planning::MotorwayInteractiveType::MERGE_IN_ROAD);
  std::vector<ConnectionConflictInfo> ans_mergein =
      cdbm.ComputeConflictMergeInData(task_info);
  bool is_main = true;
  std::vector<ConnectionConflictInfo> ans_necessary_mergein{};
  for (const auto& iter : ans_mergein) {
    bool is_necessary = false;
    if (!JudgeNecessaryMerge(task_info, iter.agent.obs_ptr,
                             iter.agent.obs_ptr->matched_lane_id(),
                             is_necessary)) {
    }
    if (is_necessary && iter.conflict_area_bound[3] > 0.0) {
      ans_necessary_mergein.emplace_back(iter);
    }
    if (iter.way_right != RightOfWay::ADVANTAGE) {
      is_main = false;
      break;
    }
  }
  // Reduce speed for obstacles processed by the map, but do not decelerate
  // sharply
  // for (auto& ans : ans_mergein) {
  //   double brake_distance = std::pow(adc_current_v, 2) / (-2.0 *
  //   constant_acc_); ans.conflict_area_bound[3] = brake_distance;
  //   ans.conflict_area_bound[2] = brake_distance + 5.0;
  // }
  if (!cibm_agents_.empty() && !is_main) {
    conflict_decision.motorway_merge_safe_decelerate = true;
    conflict_decision.motorway_free_conflict_infos = ans_mergein;
  }
  return ans_necessary_mergein;
}

bool ConflictInfoForMergeIn::EgoScenarioIsCruise(TaskInfo& task_info) {
  const auto& curr_scenario =
      DataCenter::Instance()->master_info().curr_scenario();
  if (curr_scenario != ScenarioState::MOTORWAY_CRUISE) {
    return false;
  }
  return true;
}

bool ConflictInfoForMergeIn::IgnoreObsOnVehicleFrame(
    const InsidePlannerData& inside_data, const Obstacle* const obstacle) {
  std::vector<Vec2d> obs_bounding_box_corners;
  const double adc_back_edge_to_center =
      VehicleParam::Instance()->back_edge_to_center();
  const double adc_width = VehicleParam::Instance()->width();
  obstacle->bounding_box().get_all_corners(&obs_bounding_box_corners);
  double obs_nearest_local_y{std::numeric_limits<double>::infinity()};
  double obs_max_local_x{std::numeric_limits<double>::lowest()};
  bool obs_has_negative_y{false};
  bool obs_has_positive_y{false};
  for (const auto& pt : obs_bounding_box_corners) {
    double obs_local_x{0.0}, obs_local_y{0.0}, obs_local_heading{0.0};
    earth2vehicle(inside_data.vel_x, inside_data.vel_y, inside_data.vel_heading,
                  pt.x(), pt.y(), obstacle->velocity_heading(), obs_local_x,
                  obs_local_y, obs_local_heading);
    obs_max_local_x = std::max(obs_max_local_x, obs_local_x);
    obs_nearest_local_y = std::min(obs_nearest_local_y, std::abs(obs_local_y));
    obs_has_negative_y = obs_has_negative_y || (obs_local_y < -1e-4);
    obs_has_positive_y = obs_has_positive_y || (obs_local_y > 1e-4);
  }

  if ((obs_max_local_x < -adc_back_edge_to_center) &&
      ((obs_nearest_local_y < adc_width / 2.0) ||
       (obs_has_negative_y && obs_has_positive_y))) {
    return true;
  }

  return false;
}

bool ConflictInfoForMergeIn::FilterWhenInSameLane(Obstacle* const obs,
                                                  TaskInfo& task_info) {
  if (IgnoreObsOnVehicleFrame(task_info.current_frame()->inside_planner_data(),
                              obs)) {
    LOG_INFO("obs [{}] in same lane, ignore.", obs->id());
    return true;
  }
  if (!ego_exist_turn_) return true;
  if (!ObsIsInTurnDirection(obs)) {
    LOG_INFO("obs [{}] is not in turn direction, ignore.", obs->id());
    return true;
  }

  return false;
}

bool ConflictInfoForMergeIn::JudgeSameLane(
    Obstacle* const obstacle, const ReferenceLinePtr& reference_line,
    const double& adc_current_s) {
  double min_l = 0.0;
  double max_l = 0.0;
  double max_s = 0.0;
  double min_s = -1.0;
  for (const auto& ref_point : reference_line->ref_points()) {
    if (ref_point.s() < adc_current_s - config_->back_attention_distance)
      continue;
    if (ref_point.s() > adc_current_s + config_->front_attention_distance)
      break;
    if (min_s = -1.0) {
      min_s = ref_point.s();
    }
    max_l = std::max(max_l, ref_point.left_lane_bound());
    min_l = std::min(min_l, ref_point.right_lane_bound());
    max_s = ref_point.s();
  }
  Boundary boundary{min_s, max_s, min_l, max_l};
  if (boundary.has_overlap(obstacle->PolygonBoundary())) {
    return true;
  }
  return false;
}

void ConflictInfoForMergeIn::JudgeEgoTurnStateAndDirection(
    TaskInfo& task_info) {
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();
  const double adc_front_edge_s =
      adc_current_s_ + VehicleParam::Instance()->front_edge_to_center();
  const auto& path_points = task_info.current_frame()
                                ->outside_planner_data()
                                .path_data->path()
                                .path_points();
  for (auto& pt : path_points) {
    if (pt.s() > adc_current_s_ + config_->ego_turn_attention_distance)
      continue;
    if (std::abs(pt.kappa()) > 0.05) {
      ego_exist_turn_ = true;
      if (pt.kappa() > 0.0) {
        ego_is_turn_left_ = true;
      } else {
        ego_is_turn_left_ = false;
      }
    }
  }

  LOG_INFO("ego exist turn {}, turn left {}", ego_exist_turn_,
           ego_is_turn_left_);
}

// Position criterion
bool ConflictInfoForMergeIn::ObsIsInTurnDirection(Obstacle* const obs) {
  const double adc_width = VehicleParam::Instance()->width();
  if (obs->center_sl().s() > adc_current_s_) return false;
  if (ego_is_turn_left_) {
    if (obs->center_sl().l() < adc_width) {
      return true;
    }
  } else {
    if (obs->center_sl().l() > -adc_width) {
      return true;
    }
  }
  return false;
}

bool ConflictInfoForMergeIn::JudgeNecessaryMerge(TaskInfo& task_info,
                                                 const Obstacle* obs,
                                                 const uint64_t lane_id,
                                                 bool& is_necessary) {
  bool is_left = true;
  if (task_info.adc_boundary().start_l() > obs->PolygonBoundary().start_l()) {
    is_left = false;
  }
  if (task_info.adc_boundary().end_l() < obs->PolygonBoundary().end_l()) {
    is_left = true;
  }
  if (task_info.adc_boundary().start_l() > obs->PolygonBoundary().start_l() &&
      task_info.adc_boundary().end_l() < obs->PolygonBoundary().end_l()) {
    LOG_INFO("Corner: obs's boundary is too big!");
    return false;
  }
  auto type = PlanningMap::Instance()->GetDividerType(lane_id, is_left);
  if (type == 3 || type == 4) {
    LOG_INFO("Merge in lane type is solid!");
    is_necessary = true;
  }
  return true;
}

void ConflictInfoForMergeIn::CalcRelativeLatDistance(
    TaskInfo& task_info, const Obstacle* obs, std::vector<int>& obs_ids) {
  double lat_risk_dis =
      config::PlanningConfig::Instance()
          ->planning_research_config()
          .motorway_speed_confilict_decider_config.merge_in_road_scenario_config
          .front_risk_lateral_dis;
  if (task_info.adc_boundary().start_l() > obs->PolygonBoundary().start_l()) {
    double l_diff =
        task_info.adc_boundary().start_l() - obs->PolygonBoundary().end_l();
    LOG_INFO("obs id is {}, right: l_diff is {}", obs->id(), l_diff);
    if (l_diff > 0.0 && l_diff <= lat_risk_dis) {
      obs_ids.emplace_back(obs->id());
    }
  }
  if (task_info.adc_boundary().end_l() < obs->PolygonBoundary().end_l()) {
    double l_diff =
        obs->PolygonBoundary().end_l() - task_info.adc_boundary().end_l();
    LOG_INFO("obs id is {}, left: l_diff is {}", obs->id(), l_diff);
    if (l_diff > 0.0 && l_diff <= lat_risk_dis) {
      obs_ids.emplace_back(obs->id());
    }
  }
}

}  // namespace planning
}  // namespace neodrive
