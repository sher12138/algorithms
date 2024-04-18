#include "conflict_info_for_diverging.h"

namespace neodrive {
namespace planning {

namespace {
constexpr double kKeepLength = 40.0;
}

ConflictInfoForDiverging::ConflictInfoForDiverging()
    : ConflictInfoInterface("ConflictInfoForMergeIn") {}

void ConflictInfoForDiverging::Clear() {
  adc_current_s_ = 0.0;
  adc_current_v_ = 0.0;
  adc_front_edge_s_ = 0.0;
  custom_agents_.clear();
}

void ConflictInfoForDiverging::Init(TaskInfo& task_info) {
  Clear();
  adc_current_s_ = task_info.curr_sl().s();
  adc_front_edge_s_ = VehicleParam::Instance()->front_edge_to_center();
  adc_current_v_ = task_info.current_frame()->inside_planner_data().vel_v;
}

std::vector<ConnectionConflictInfo>
ConflictInfoForDiverging::ComputeConflictInfo(TaskInfo& task_info) {
  if (!config::PlanningConfig::Instance()
           ->planning_research_config()
           .motorway_speed_confilict_decider_config
           .diverging_road_scenario_config.diverging_conflict_enable) {
    LOG_INFO("diverging_conflict_enable is false , skip!");
    return {};
  }
  Init(task_info);

  ConflictDataBasePath cdbp;
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;
  const bool ego_in_diverging =
      (traffic_conflict_zone_context.near_type ==
           TrafficConflictZoneContext::connectionType::Diverging ||
       traffic_conflict_zone_context.near_type ==
           TrafficConflictZoneContext::connectionType::NDiverging)
          ? true
          : false;

  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  const auto& dynamic_obstacle = task_info.current_frame()
                                     ->planning_data()
                                     .decision_data()
                                     .dynamic_obstacle();

  if (speed_planner_common::EgoInIntersection(task_info)) {
    LOG_INFO("ego in intersection , skip!");
    return {};
  }
  if (speed_planner_common::EgoInTParkCross(task_info)) {
    LOG_INFO("ego in park cross , skip!");
    return {};
  }
  if (speed_planner_common::EgoInTNonParkCross(task_info)) {
    LOG_INFO("ego in T Cross , skip!");
    return {};
  }

  if (ego_in_diverging && traffic_conflict_zone_context.distance_to_zone >
                              config_->max_attention_distance) {
    LOG_INFO("distance_to_zone > 50 , skip!");
    return {};
  }

  // don't triggle diverging, skip
  if (!DivergingTriggleJudge(task_info, ego_in_diverging)) {
    LOG_INFO("DivergingTriggleJudge is false , skip!");
    return {};
  }

  GenerateEgoConsiderDirection(task_info, traffic_conflict_zone_context,
                               ego_in_diverging);
  // judge curve current lane,rf ego curve change rate 
  LOG_INFO("ego consider direction {}", ego_consider_direction_);
  for (const auto& obs : dynamic_obstacle) {
    if (cdbp.CheckCollisionForDiverging(task_info, *obs)) {
      LOG_INFO("Initial collision between obstacles and the vehicle, id is {}!",
               obs->id());
      continue;
    }
    if (obs->type() == Obstacle::ObstacleType::PEDESTRIAN) {
      LOG_INFO("obs's type is pedeatrian, id is {}!", obs->id());
      continue;
    }
    if (obs->type() == Obstacle::ObstacleType::BICYCLE && obs->speed() < 5.5) {
      LOG_INFO("obs's type is bicycle, id is {}, speed is {}!", obs->id(),
               obs->speed());
      continue;
    }
    if (obs->min_s() > adc_current_s_ + adc_front_edge_s_) {
      LOG_INFO("obs is too far away, obs id:{},{:.3f},{:.3f}", obs->id(),
               obs->min_s(), adc_current_s_ + adc_front_edge_s_);
      continue;
    }
    if (ego_in_diverging && LowerRiskObsIgnore(task_info, *obs)) {
      LOG_INFO("obs {} is low risk, ignore", obs->id());
      continue;
    }
    if (FilterSolution(*obs, task_info)) {
      continue;
    }

    if ((task_info.curr_sl().l() - obs->center_sl().l() < 0.) &&
        (ego_consider_direction_ == 1)) {
      custom_agents_.insert(obs->id());
      LOG_INFO("add left turn obs id:{}", obs->id());
      continue;
    };
    if ((task_info.curr_sl().l() - obs->center_sl().l() > -0.) &&
        (ego_consider_direction_ == 2)) {
      custom_agents_.insert(obs->id());
      LOG_INFO("add right turn obs id:{}", obs->id());
      continue;
    }
    if (ego_consider_direction_ == 3) {
      custom_agents_.insert(obs->id());
      LOG_INFO("error lane theta change, add all obs:{}", obs->id());
    }
    LOG_INFO("MIX! obs id:{}", obs->id());
  }
  LOG_INFO("custom_agents_ size:{}", custom_agents_.size());
  cdbp.set_interactive_agent_ids(
      custom_agents_, neodrive::planning::MotorwayInteractiveType::CUSTOM);
  std::vector<ConnectionConflictInfo> ans_custom =
      cdbp.ComputeConflictCustomData(task_info);

  return ans_custom;
}

bool ConflictInfoForDiverging::DivergingTriggleJudge(
    TaskInfo& task_info, const bool ego_in_diverging) {
  LOG_INFO(
      "ego_in_diverging:{},last_is_diverging_:{},change_sc_sign_:{},"
      "keep_length_:{:.3f},adc_current_s:{:.3f},start_point_s:{:.3f},last_skip_"
      "reason_:{}",
      ego_in_diverging, last_is_diverging_, change_sc_sign_, keep_length_,
      adc_current_s_, start_s_, static_cast<int>(last_skip_reason_));
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;

  if (ego_in_diverging) {
    if (last_is_diverging_) {
      last_skip_reason_ = traffic_conflict_zone_context.type;
      LOG_INFO("setting dealy zone type:{}",
               static_cast<int>(last_skip_reason_));
    }
    return true;
  }
  if (!last_is_diverging_ ||
      (last_is_diverging_ && change_sc_sign_ &&
       keep_length_ < task_info.curr_sl().s() - start_s_)) {
    LOG_INFO("ego not in diverging  and last_is_diverging_ is false, skip!");
    keep_length_ = 0.0;
    acculate_dis_ = 0.0;
    start_point_ = Vec2d{0.0, 0.0};
    start_s_ = -1.0;
    last_is_diverging_ = false;
    change_sc_sign_ = false;
    ego_consider_direction_ = 0;
    history_directory_ = 0;
    return false;
  }

  if (last_is_diverging_ && !change_sc_sign_) {
    keep_length_ = config_->delay_zone_length;
    start_point_ = Vec2d{task_info.adc_point().x(), task_info.adc_point().y()};
    change_sc_sign_ = true;
    // update delay zone
    const auto& ref_line = task_info.reference_line();
    SLPoint sl_point;
    ref_line->GetPointInFrenetFrame(start_point_, &sl_point);
    start_s_ = sl_point.s();
    LOG_INFO(
        "ego come in delay zone , keep_length:{:.3f},start_point:[{:.3f}, "
        "{:.3f}]",
        keep_length_, start_point_.x(), start_point_.y());
    return true;
  }
  const auto& ref_line = task_info.reference_line();
  SLPoint sl_point;
  ref_line->GetPointInFrenetFrame(start_point_, &sl_point);
  start_s_ = sl_point.s();

  if (keep_length_ >= task_info.curr_sl().s() - start_s_) {
    LOG_INFO("ego in delay zone, keep!");
    return true;
  }
  return false;
}

bool ConflictInfoForDiverging::FilterSolution(Obstacle& obs,
                                              TaskInfo& task_info) {
  // ego
  const auto& inside_planner_data =
      task_info.current_frame()->inside_planner_data();
  const double& ego_vel = inside_planner_data.vel_v;
  const double& ego_heading = inside_planner_data.vel_heading;
  if (std::abs(normalize_angle(obs.velocity_heading() -
                               inside_planner_data.vel_heading)) >
      config_->max_attention_heading_diff) {
    LOG_INFO("ignore by heading_diff, obs id:{}", obs.id());
    return true;
  }
  // speed diff
  if (obs.speed() - ego_vel < config_->min_valid_speed_diff &&
      obs.speed() - ego_vel > 0.0) {
    LOG_INFO("ignore by speed_diff, obs id:{}", obs.id());
    return true;
  }
  // distance is bigger
  if (std::hypot(obs.center().x() - task_info.adc_point().x(),
                 obs.center().y() - task_info.adc_point().y()) >
      config_->max_valid_xy_diff) {
    LOG_INFO("ignore by distance, obs id:{}", obs.id());
    return true;
  }
  return false;
}

void ConflictInfoForDiverging::GenerateEgoConsiderDirection(
    TaskInfo& task_info,
    const TrafficConflictZoneContext& traffic_conflict_zone_context,
    const bool ego_in_diverging) {
  if (ego_in_diverging && last_is_diverging_) {
    last_skip_reason_ = traffic_conflict_zone_context.type;
    history_directory_ = ego_consider_direction_;
    LOG_INFO("setting dealy zone type:{}", static_cast<int>(last_skip_reason_));
  }
  // in handle, keep diverging
  last_is_diverging_ = true;
  if (keep_length_ > kKeepLength) {
    keep_length_ == kKeepLength;
  }
  if (ego_in_diverging) {
    change_sc_sign_ = false;
  }
  // ego not in diverging and in delay zone , delta{s} > kee_length valid !
  if (!ego_in_diverging && adc_current_s_ - start_s_ > keep_length_) {
    last_is_diverging_ = false;
    LOG_INFO("ego out of delay zone ,{}, keep_length:{:.3f},start_s_:{:.3f}",
             task_info.curr_sl().s(), keep_length_, start_s_);
  }

  const auto& path_data =
      task_info.current_frame()->outside_planner_data().path_data;

  // GetEgoConsiderDirection();
  const auto& successor_ids = traffic_conflict_zone_context.successor_ids;
  LOG_INFO("successor_ids size:{}", successor_ids.size());

  if (ego_in_diverging) {
    std::unordered_set<uint64_t> ref_lanes{};
    const auto& ref_line = task_info.reference_line();
    for (const auto& ref_pt : ref_line->ref_points()) {
      if (ref_pt.s() > adc_current_s_ + config_->front_view_distance) {
        break;
      }
      if (ref_pt.s() < adc_current_s_) {
        continue;
      }

      ref_lanes.insert(ref_pt.hd_map_lane_id());
    }
    // ref_success lane
    std::unordered_set<uint64_t> left_lane{}, right_lane{};
    for (auto& lane : successor_ids) {
      LOG_INFO("lane steering :{:.3f}, lane.id:{}", lane.steering, lane.id);
      if (lane.is_extended) {
        LOG_INFO("lane id:{}, is_extended!,skip!", lane.id);
        continue;
      }
      if (lane.steering > 0.0) {
        left_lane.insert(lane.id);
      }  // left
      else {
        right_lane.insert(lane.id);
      }  // right
    }
    for (const auto& ref_lane : ref_lanes) {
      LOG_INFO("ref_line lane id {}", ref_lane);
      if (left_lane.find(ref_lane) != left_lane.end()) {
        ego_consider_direction_ = 1;
        break;
      } else if (right_lane.find(ref_lane) != right_lane.end()) {
        ego_consider_direction_ = 2;
        break;
      }
    }
    if (ego_consider_direction_ == 0) {
      LOG_ERROR("ego successor lane is error!")
    }
  }

  if (history_directory_ != ego_consider_direction_ &&
      history_directory_ != 0) {
    ego_consider_direction_ = 3;
  }
}

bool ConflictInfoForDiverging::LowerRiskObsIgnore(TaskInfo& task_info,
                                                  const Obstacle& obstalce) {
  const auto& traffic_conflict_zone_context =
      task_info.current_frame()
          ->outside_planner_data()
          .traffic_conflict_zone_context;

  const double ego_to_diverging_distance =
      traffic_conflict_zone_context.distance_to_zone;
  const double ego_to_obs_l =
      std::min(abs(obstalce.center_sl().l() - task_info.curr_sl().l()),
               abs(obstalce.center_sl().l() - task_info.curr_sl().l()));
  const double ego_to_obs =
      task_info.adc_point().distance_to(obstalce.center()) -
      VehicleParam::Instance()->width() * 0.25 - obstalce.length() * 0.25;
  const double heading_diff = normalize_angle(
      obstalce.velocity_heading() -
      task_info.current_frame()->inside_planner_data().vel_heading);
  if (!(std::abs(heading_diff) < M_PI * 0.25 ||
        std::abs(heading_diff) > M_PI * 3.0 * 0.25) &&
      ego_to_obs / obstalce.speed() > config_->safe_thw) {
    LOG_INFO("ego_to_obs {:.3f}, obs speed {:.3f}, safe_thw {:.3f}, ignore!",
             ego_to_obs, obstalce.speed(), config_->safe_thw);
    return true;
  }

  // alone to handle obs is small degree vehicle

  const double stop_dis = 0.5 * std::pow(adc_current_v_, 2) / config_->deacc;

  if (ego_to_diverging_distance > stop_dis &&
      ego_to_obs_l > config_->adopt_lateral_distance) {
    LOG_INFO(
        "ego_to_obs l {:.3f}, ego_to_diverging_distance {:.3f}, stop_dis "
        "{:.3f}, "
        "adopt_lateral_distance {:.3f}, ignore!",
        ego_to_obs, ego_to_diverging_distance, stop_dis,
        config_->adopt_lateral_distance);
    return true;
  }
  if (ego_to_diverging_distance < stop_dis &&
      ego_to_obs_l > ego_to_diverging_distance / stop_dis *
                         config_->adopt_lateral_distance * 3.0) {
    LOG_INFO(
        "ego_to_obs l {:.3f}, ego_to_diverging_distance {:.3f}, stop_dis "
        "{:.3f}, "
        "adopt_lateral_distance {:.3f}, ignore!",
        ego_to_obs_l, ego_to_diverging_distance, stop_dis,
        config_->adopt_lateral_distance);
    return true;
  }

  return false;
}
}  // namespace planning
}  // namespace neodrive