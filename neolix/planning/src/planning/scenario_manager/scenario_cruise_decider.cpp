#include "scenario_cruise_decider.h"

namespace neodrive {
namespace planning {

ScenarioCruiseDecider::ScenarioCruiseDecider() {
  name_ = "ScenarioCruiseDecider";
}

bool ScenarioCruiseDecider::Init() {
  if (initialized_) {
    return true;
  }

  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_cruise_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioCruiseDecider init.");
  return true;
}

bool ScenarioCruiseDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioCruiseDecider.");
  return true;
}

ErrorCode ScenarioCruiseDecider::RunOnce() {
  LOG_INFO("ScenarioCruiseDecider::RunOnce");
  LOG_INFO("Cruise cur stage: {}", curr_state());
  // change stage.
  is_state_change_ = false;
  ComputeAttentionInfos();
  auto it = state_machine_function_map_.find(curr_state());
  if (it != state_machine_function_map_.end()) {
    auto func = it->second;
    (this->*func)();
    auto &task_info = data_center_->mutable_task_info_list()->front();
    scenario_common::RefLFilter(task_info);
  } else {
    // missing handle state function
    LOG_ERROR("missing handle state function {}", curr_state_str_);
  }
  if (is_state_change_) {
    if (!UpdateStateMachine()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  SaveMonitorMessage();
  return ErrorCode::PLANNING_OK;
}

bool ScenarioCruiseDecider::ResetStateMachine() {
  if (!state_machine_.SetInitializeState("KEEP")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(
      static_cast<CruiseStageState::State>(state_machine_.GetCurrentState()));
  prev_decision_ = CruiseStageChangeFlag::T0_NONE_KEEP;
  curr_decision_ = CruiseStageChangeFlag::T0_NONE_KEEP;
  return true;
}

void ScenarioCruiseDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::KEEP, &ScenarioCruiseDecider::OnHandleStateKeep));
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::CENTERING,
          &ScenarioCruiseDecider::OnHandleStateCentering));
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::BIAS_DRIVE,
          &ScenarioCruiseDecider::OnHandleStateBiasDrive));
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::RIGHT_AVOID,
          &ScenarioCruiseDecider::OnHandleStateRightAvoid));
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::LEFT_OVERTAKE,
          &ScenarioCruiseDecider::OnHandleStateLeftOvertake));
  state_machine_function_map_.insert(
      std::pair<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>(
          CruiseStageState::PULL_OVER,
          &ScenarioCruiseDecider::OnHandleStatePullOver));
}

bool ScenarioCruiseDecider::UpdateStateMachine() {
  // filter decision.
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  } else if (curr_state() != CruiseStageState::KEEP) {
    filter_decision_ = curr_decision_;
    decision_filter_count = cruise_scenario_conf_.filter_frame_threshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             CruiseStageChangeFlag::ChangeFlag_Name(curr_decision_));
  }
  if (decision_filter_count < cruise_scenario_conf_.filter_frame_threshold) {
    // hold state.
    LOG_INFO("hold state: {}, decision_filter_count: {}", curr_state(),
             decision_filter_count);
    return true;
  }
  // update state.
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR("{}: state change fail. state: {}, changeflag: {}", name_,
              curr_state_str_,
              CruiseStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(
        static_cast<CruiseStageState::State>(state_machine_.GetCurrentState()));
    prev_decision_ = curr_decision_;
    if (curr_state_str_ != prev_state) {
      prev_state_str_ = prev_state;
      LOG_INFO("{}: update cruise stage: ({}) {} {}", Name(),
               state_machine_.GetChangeFlagStr(curr_decision_), prev_state,
               curr_state_str_);
    }
  }
  return true;
}

void ScenarioCruiseDecider::OnHandleStateKeep() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  if (task_info.last_frame() != nullptr) {
    observe_ref_l =
        task_info.last_frame()->inside_planner_data().init_sl_point.l();
  }

  curr_decision_ = CruiseStageChangeFlag::T0_NONE_KEEP;
  if (!CheckKeepFinished()) {
    is_state_change_ = false;
    return;
  }
  if (CheckPullOverTriggered()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T9_KEEP_PULL_OVER;
    return;
  }
  if (CheckCenteringTriggered()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T1_KEEP_CENTERING;
    return;
  }
  if (CheckBiasDriveTriggered()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T3_KEEP_BIAS_DRIVE;
    return;
  }
  if (CheckLeftOvertakeTriggered()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T7_KEEP_LEFT_OVERTAKE;
    return;
  }
  if (CheckRightAvoidTriggered()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T5_KEEP_RIGHT_AVOID;
    return;
  }
}

void ScenarioCruiseDecider::OnHandleStateCentering() {
  if (CheckCenteringFinished()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T2_CENTERING_KEEP;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  scenario_common::ComputeRefL(task_info, false);
  return;
}

void ScenarioCruiseDecider::OnHandleStateBiasDrive() {
  if (CheckBiasDriveFinished()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T4_BIAS_DRIVE_KEEP;
    return;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, true);
}

void ScenarioCruiseDecider::OnHandleStateRightAvoid() {
  if (CheckRightAvoidFinished()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T6_RIGHT_AVOID_KEEP;
    return;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) {
    return;
  }
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  auto shrink_bounds_info =
      task_info.last_frame()->outside_planner_data().shrink_half_ego_boundaries;
  if (shrink_bounds_info.empty()) {
    observe_ref_l = inside_data.init_sl_point.l();
    return;
  }
  auto right_most_boundry_point = shrink_bounds_info.front();
  auto left_most_boundry_point = shrink_bounds_info.front();
  for (auto &point : shrink_bounds_info) {
    if (point.lower_type != PathRegion::Bound::BoundType::VIR) {
      if (right_most_boundry_point.lower_type ==
              PathRegion::Bound::BoundType::VIR ||
          point.lower_point.l() > right_most_boundry_point.lower_point.l()) {
        right_most_boundry_point = point;
      }
    }
    if (point.upper_type != PathRegion::Bound::BoundType::VIR) {
      if (left_most_boundry_point.upper_type ==
              PathRegion::Bound::BoundType::VIR ||
          point.upper_point.l() < left_most_boundry_point.upper_point.l()) {
        left_most_boundry_point = point;
      }
    }
  }

  double max_width = 0.0;
  for (auto &obs : far_obstacles_) {
    max_width = std::max(max_width, obs.width());
  }
  for (auto &obs : near_obstacles_) {
    max_width = std::max(max_width, obs.width());
  }
  observe_ref_l =
      std::min(std::max(right_most_boundry_point.lower_point.l() +
                            cruise_scenario_conf_.right_road_buff,
                        left_most_boundry_point.upper_point.l() - max_width -
                            cruise_scenario_conf_.right_avoid_buff),
               inside_data.init_sl_point.l());
  LOG_INFO(
      "lower_point l: {:.4f}, upper_point l: {:.4f}, right_road_buff: "
      "{:.4f}, max_width: {:.4f}, right_avoid_buff: {:.4f}, observe_ref_l: "
      "{:.4f}",
      right_most_boundry_point.lower_point.l(),
      left_most_boundry_point.upper_point.l(),
      cruise_scenario_conf_.right_road_buff, max_width,
      cruise_scenario_conf_.right_avoid_buff, observe_ref_l);
}

void ScenarioCruiseDecider::OnHandleStateLeftOvertake() {
  if (CheckLeftOvertakeFinished()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T8_LEFT_OVERTAKE_KEEP;
    return;
  }

  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) return;

  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  const auto &outside_data = task_info.last_frame()->outside_planner_data();
  const auto &shrink_bounds_info = outside_data.shrink_half_ego_boundaries;
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  if (shrink_bounds_info.empty()) {
    observe_ref_l = inside_data.init_sl_point.l();
    return;
  }
  double left_l = 100.0;
  double right_l = -100.0;
  for (auto &bound : shrink_bounds_info) {
    if (bound.upper_type != PathRegion::Bound::BoundType::VIR)
      left_l = std::min(left_l, bound.upper_point.l());
    if (bound.lower_type != PathRegion::Bound::BoundType::VIR)
      right_l = std::max(right_l, bound.lower_point.l());
  }
  LOG_INFO("left_l: {:.4f}, right_l: {:.4f}", left_l, right_l);

  if (only_front_slow_obs_) {
    observe_ref_l =
        std::max(inside_data.init_sl_point.l(),
                 std::min(left_l - cruise_scenario_conf_.left_road_buff,
                          front_slow_obs_max_l_ +
                              0.5 * VehicleParam::Instance()->width() +
                              cruise_scenario_conf_.left_overtake_buff));
    LOG_INFO("go left overtake");
  }
  if (only_near_right_front_obs_) {
    observe_ref_l =
        std::max(inside_data.init_sl_point.l(),
                 std::min(left_l - cruise_scenario_conf_.left_road_buff,
                          near_front_slow_obs_max_l_ +
                              0.5 * VehicleParam::Instance()->width() +
                              cruise_scenario_conf_.left_overtake_buff));
    LOG_INFO("near_front_slow_obs_max_l: {:.4f}", near_front_slow_obs_max_l_);
    LOG_INFO("near obs go left overtake");
  }
}

void ScenarioCruiseDecider::OnHandleStatePullOver() {
  if (CheckPullOverFinished()) {
    is_state_change_ = true;
    curr_decision_ = CruiseStageChangeFlag::T10_PULL_OVER_KEEP;
  }
  return;
}

bool ScenarioCruiseDecider::IsFarway(const Obstacle &obstacle,
                                     const Boundary &adc_boundary) {
  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  if (obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s() ||
      obstacle.max_s() + ref_conf.observe_back_dis < adc_boundary.start_s() ||
      obstacle.min_l() - ref_conf.observe_left_dis > adc_boundary.end_l() ||
      obstacle.max_l() + ref_conf.observe_right_dis < adc_boundary.start_l()) {
    LOG_INFO(
        "skip faraway obs id[{}], front[{}] back[{}] left[{}] right[{}].",
        obstacle.id(),
        obstacle.min_s() - ref_conf.observe_front_dis > adc_boundary.end_s(),
        obstacle.max_s() +
            ref_conf
                .observe_back_dis<adc_boundary.start_s(),
                                  obstacle.min_l() - ref_conf.observe_left_dis>
                    adc_boundary.end_l(),
        obstacle.max_l() + ref_conf.observe_right_dis < adc_boundary.start_l());
    return false;
  } else {
    return true;
  }
}

bool ScenarioCruiseDecider::IsInbound(
    const Obstacle &obstacle,
    const std::vector<neodrive::planning::PathRegion::Bound> &bounds,
    double &obs_left_witdh, double &obs_right_witdh) {
  if (bounds.empty()) return false;

  double begin_delt_s =
      (obstacle.min_s() - bounds.front().lower_point.s()) / 0.1;
  double end_delt_s = (obstacle.max_s() - bounds.front().lower_point.s()) / 0.1;

  size_t start_index = 0;
  while (start_index < bounds.size()) {
    if (bounds[start_index].lower_type != PathRegion::Bound::BoundType::VIR &&
        bounds[start_index].upper_type != PathRegion::Bound::BoundType::VIR)
      break;
    ++start_index;
  }

  size_t end_index = std::clamp(static_cast<size_t>(std::max(0.0, end_delt_s)),
                                start_index, bounds.size() - 1);
  size_t begin_index =
      std::clamp(static_cast<size_t>(std::max(0.0, begin_delt_s)), start_index,
                 bounds.size() - 1);
  LOG_INFO(
      "start_index: {}, search begin: {}, end: {}, bounds size: {}. bounds s: "
      "{:.4f}, begin_delt_s: {:.4f}, end_delt_s: {:.4f}",
      start_index, begin_index, end_index, bounds.size(),
      bounds.front().lower_point.s(), begin_delt_s, end_delt_s);
  if (begin_index <= end_index) {
    double max_upper_l = 0.0, min_upper_l = 100.0;
    double max_lower_l = -100.0, min_lower_l = 0;
    // Screen obstacles based on the bounds
    for (size_t i = begin_index; i <= end_index; ++i) {
      max_upper_l = std::max(max_upper_l, bounds[i].upper_point.l());
      min_upper_l = std::min(min_upper_l, bounds[i].upper_point.l());
      max_lower_l = std::max(max_lower_l, bounds[i].lower_point.l());
      min_lower_l = std::min(min_lower_l, bounds[i].lower_point.l());
    }
    obs_left_witdh = min_upper_l - obstacle.max_l();
    obs_right_witdh = obstacle.min_l() - max_lower_l;
    LOG_INFO(
        "lower_point l: {:.4f}, upper_point l: {:.4f}, obstacle.min_l: "
        "{:.4f}, obstacle.max_l(): {:.4f}, obs_left_witdh: {:.4f}, "
        "obs_right_witdh: {:.4f}",
        max_lower_l, min_upper_l, obstacle.min_l(), obstacle.max_l(),
        obs_left_witdh, obs_right_witdh);
    if (obstacle.min_l() > max_upper_l + 0.5 ||
        obstacle.max_l() < min_lower_l - 0.5) {
      LOG_INFO("skip obs [{}] not in lane bound", obstacle.id());
      return false;
    }
  } else {
    LOG_INFO("skip obs [{}] not in lane bound", obstacle.id());
    return false;
  }
  return true;
}

bool ScenarioCruiseDecider::ComputeAttentionInfos() {
  near_obstacles_.clear();
  far_obstacles_.clear();
  front_slow_obs_max_l_ = -100.0;
  only_near_right_front_obs_ = false;
  if (!non_motorway_config_.enable_dynamic_obs_detour ||
      !data_center_->master_info().enable_static_detour()) {
    return false;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) {
    LOG_INFO("last_frame null");
    return false;
  }
  if (task_info.last_frame()->inside_planner_data().is_in_the_park) {
    LOG_INFO("Ego is in the park, dynamic detour is disabled");
    return false;
  }
  auto decision_data =
      task_info.last_frame()->planning_data().mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return false;
  }

  auto &ref_ptr = task_info.reference_line();
  if (ref_ptr == nullptr || ref_ptr->ref_points().empty()) {
    return false;
  }

  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  const auto &outside_data = task_info.last_frame()->outside_planner_data();

  const auto &original_path_boundary =
      outside_data.path_context.original_path_boundary.path_boundary;

  const auto &adc_boundary = outside_data.path_obstacle_context.adc_boundary;

  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  const float &front_dis = ref_conf.danger_range.front_dis;
  const float &back_dis = ref_conf.danger_range.back_dis;
  const float &left_dis = ref_conf.danger_range.left_dis;
  const float &right_dis = ref_conf.danger_range.right_dis;

  target_s_ = inside_data.init_sl_point.s() + 100.0;
  only_front_slow_obs_ = false;

  double k = config::PlanningConfig::Instance()
                 ->planning_research_config()
                 .path_observe_ref_decider_config.vhe_v_filter_ratio;
  veh_v_ = veh_v_ * (1.0 - k) +
           k * task_info.last_frame()->inside_planner_data().vel_v;
  LOG_INFO(
      "adc min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
      "{:.4f}, v: {:.4f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), veh_v_);
  // Screening obstacles
  LOG_INFO("dynamic_obstacle size: {}",
           decision_data->dynamic_obstacle().size());
  for (size_t i = 0; i < decision_data->dynamic_obstacle().size(); ++i) {
    auto obstacle = *decision_data->dynamic_obstacle()[i];
    LOG_INFO(
        "obstacle[{}] min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
        "{:.4f}, v: {:.4f}",
        obstacle.id(), obstacle.min_s(), obstacle.max_s(), obstacle.min_l(),
        obstacle.max_l(), obstacle.speed());
    // faraway obs
    if (!IsFarway(obstacle, adc_boundary)) {
      continue;
    }

    // in bound?
    double obs_left_witdh = 0.0;
    double obs_right_witdh = 0.0;
    if (!IsInbound(obstacle, original_path_boundary, obs_left_witdh,
                   obs_right_witdh)) {
      continue;
    }

    /***/
    ReferencePoint reference_point;
    if (!ref_ptr->GetNearestRefPoint(obstacle.center_sl().s(),
                                     &reference_point)) {
      LOG_INFO("GetNearestRefPoint fail");
      continue;
    }
    double obs_heading = obstacle.velocity_heading();
    double heading_diff = std::abs(normalize_angle(obstacle.velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > ref_conf.filter_obs_heading_threshold &&
        M_PI - heading_diff > ref_conf.filter_obs_heading_threshold) {
      LOG_INFO("skip oblique obstacle[{}] heading_diff: {:.4f}", obstacle.id(),
               heading_diff);
      continue;
    }
    if (heading_diff > M_PI / 2.0 && obs_right_witdh < 0.0) {
      LOG_INFO("skip right obverse obstacle[{}] obs_right_witdh: {:.4f}",
               obstacle.id(), obs_right_witdh);
      continue;
    }

    if (!(obstacle.min_s() - front_dis > adc_boundary.end_s() ||
          obstacle.max_s() + back_dis < adc_boundary.start_s() ||
          obstacle.min_l() - left_dis > adc_boundary.end_l() ||
          obstacle.max_l() + right_dis < adc_boundary.start_l())) {
      if (heading_diff > M_PI / 2.0 &&
          obstacle.max_s() + 1.0 < adc_boundary.start_s()) {
        LOG_INFO("skip obverse back obstacle[{}] heading_diff: {:.4f}",
                 obstacle.id(), heading_diff);
        continue;
      }
      LOG_INFO("find near obs[{}], front[{}] back[{}] left[{}] right[{}].",
               obstacle.id(),
               obstacle.min_s() - front_dis > adc_boundary.end_s(),
               obstacle.max_s() +
                   back_dis<adc_boundary.start_s(), obstacle.min_l() - left_dis>
                       adc_boundary.end_l(),
               obstacle.max_l() + right_dis < adc_boundary.start_l());
      near_obstacles_.push_back(obstacle);
      continue;
    }

    if (heading_diff < M_PI / 2.0) {
      // Same direction
      // Establishing an artificial potential field
      double max_v = std::max(veh_v_, 3.0);
      if (obstacle.min_s() > adc_boundary.end_s() && obstacle.speed() > max_v) {
        LOG_INFO(
            "skip front high speed obs[{}]: obs speed: {:.4f}, veh_v_: "
            "{:.4f}",
            obstacle.id(), obstacle.speed(), max_v);
        continue;
      }
      if (obstacle.max_s() < adc_boundary.start_s() &&
          obstacle.speed() < veh_v_) {
        LOG_INFO(
            "skip back low speed obs[{}]: obs speed: {:.4f}, veh_v_: {:.4f}",
            obstacle.id(), obstacle.speed(), veh_v_);
        continue;
      }
      if (obstacle.min_s() > adc_boundary.end_s() && obstacle.speed() < max_v) {
        double delt_s = obstacle.min_s() - front_dis - adc_boundary.end_s();
        double delt_v = max_v - obstacle.speed();
        double delt_t = delt_s / delt_v;
        if (delt_t > 8.0 || delt_t < 0.0 || inside_data.is_lane_borrowing) {
          LOG_INFO(
              "skip front obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}, is_lane_borrowing: {}",
              obstacle.id(), delt_s, delt_v, delt_t,
              inside_data.is_lane_borrowing);
          continue;
        } else {
          if (((obs_left_witdh > obs_right_witdh && obs_right_witdh < 2.0) ||
               obs_left_witdh > 2.5) &&
              (far_obstacles_.empty() ||
               (!far_obstacles_.empty() && only_front_slow_obs_))) {
            target_s_ = inside_data.init_sl_point.s() + max_v * delt_t;
            only_front_slow_obs_ = true;
            front_slow_obs_max_l_ =
                std::max(front_slow_obs_max_l_, obstacle.max_l());
          } else {
            only_front_slow_obs_ = false;
            obstacle.set_width(obs_left_witdh + obstacle.width());
            LOG_INFO("modify obs width: {:.4f}, obs_left_witdh: {:.4f}",
                     obstacle.width(), obs_left_witdh);
          }
          LOG_INFO(
              "take over obs[{}] {}, front_slow_obs_max_l: {:.4f}, "
              "obs_right_witdh: {:.4f}, obs_left_witdh: {:.4f}",
              obstacle.id(), only_front_slow_obs_, front_slow_obs_max_l_,
              obs_right_witdh, obs_left_witdh);
          LOG_INFO(
              "target_s_:{:.4f}, init s : {:.4f}, veh_v_: {:.4f}, time: "
              "{:.4f}, "
              "delt_s: {:.4f}, delt_v: {:.4f}",
              target_s_, inside_data.init_sl_point.s(), max_v, delt_t, delt_s,
              delt_v);
        }
      }
      if (obstacle.max_s() < adc_boundary.start_s() &&
          obstacle.speed() > veh_v_) {
        double delt_s = adc_boundary.start_s() - back_dis - obstacle.max_s();
        double delt_v = obstacle.speed() - veh_v_;
        double delt_t = delt_s / delt_v;
        if (delt_t > 8.0 || delt_t < 0.0) {
          LOG_INFO(
              "skip back obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }
        if (delt_t < 1.0) {
          near_obstacles_.push_back(obstacle);
          LOG_INFO(
              "find back dangerous obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
              "delt_t: {:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }

        only_front_slow_obs_ = false;
        LOG_INFO(
            "init s : {:.4f}, veh_v_: {:.4f}, time: {:.4f}, "
            "delt_s: {:.4f}, delt_v: {:.4f}",
            inside_data.init_sl_point.s(), veh_v_, delt_t, delt_s, delt_v);
      }
    } else {
      // Opposite direction
      if (obstacle.max_s() < adc_boundary.start_s()) {
        LOG_INFO("reverse back obs[{}]: obs max_s: {:.4f}, veh start_s: {:.4f}",
                 obstacle.id(), obstacle.max_s(), adc_boundary.start_s());
        continue;
      }
      double delt_s = obstacle.min_s() - adc_boundary.end_s() - back_dis;
      double delt_v = obstacle.speed() + veh_v_;
      double delt_t = delt_s / delt_v;
      if (delt_t > 8.0 || delt_t < 0.0) {
        LOG_INFO(
            "skip front reverse obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        continue;
      }
      if (delt_t < 1.0) {
        near_obstacles_.push_back(obstacle);
        LOG_INFO(
            "find back dangerous obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        continue;
      }

      only_front_slow_obs_ = false;
      LOG_INFO(
          "init s : {:.4f}, veh_v_: {:.4f}, time: {:.4f}, "
          "delt_s: {:.4f}, delt_v: {:.4f}",
          inside_data.init_sl_point.s(), veh_v_, delt_t, delt_s, delt_v);
    }
    LOG_INFO("find far obs[{}]", obstacle.id());
    far_obstacles_.push_back(obstacle);
  }
  LOG_INFO("near_obstacles_ size: {}, far_obstacles_: {}",
           near_obstacles_.size(), far_obstacles_.size());

  if (far_obstacles_.empty() && !near_obstacles_.empty()) {
    const auto &shrink_bounds_info =
        outside_data.path_context.shrink_path_boundary.path_boundary;
    double left_l = 100.0;
    double right_l = -100.0;
    for (auto &bound : shrink_bounds_info) {
      if (bound.upper_type != PathRegion::Bound::BoundType::VIR)
        left_l = std::min(left_l, bound.upper_point.l());
      if (bound.lower_type != PathRegion::Bound::BoundType::VIR)
        right_l = std::max(right_l, bound.lower_point.l());
    }
    near_front_slow_obs_max_l_ = right_l;
    ReferencePoint init_reference_point;
    for (auto &obs : near_obstacles_) {
      if (!ref_ptr->GetNearestRefPoint(obs.center_sl().s(),
                                       &init_reference_point)) {
        LOG_INFO("GetNearestRefPoint fail");
        return false;
      }
      double heading_diff = std::abs(normalize_angle(
          obs.velocity_heading() - init_reference_point.heading()));
      if (obs.min_s() > adc_boundary.end_s() &&
          obs.speed() < std::max(veh_v_, 3.0) && heading_diff < M_PI / 2.0 &&
          obs.max_l() < inside_data.init_sl_point.l() &&
          (left_l + 0.5 * VehicleParam::Instance()->width() - obs.max_l() >
           2.0)) {
        near_front_slow_obs_max_l_ =
            std::max(near_front_slow_obs_max_l_, obs.max_l());
        only_near_right_front_obs_ = true;
      } else {
        only_near_right_front_obs_ = false;
        LOG_INFO(
            "obs[{}] not takeover. speed: {}, width: {}", obs.id(),
            obs.speed() < std::max(veh_v_, 3.0),
            (left_l + 0.5 * VehicleParam::Instance()->width() - obs.max_l() >
             2.0));
        return true;
      }
    }
  }
  return true;
}

bool ScenarioCruiseDecider::CheckKeepFinished() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) return false;
  const auto &outside_data = task_info.last_frame()->outside_planner_data();
  const auto &adc_boundary = outside_data.path_obstacle_context.adc_boundary;

  if (far_obstacles_.empty() && only_near_right_front_obs_) {
    return true;
  }
  for (auto &obs : near_obstacles_) {
    if ((obs.min_l() < adc_boundary.start_l() ||
         obs.max_l() < adc_boundary.end_l())) {
      return false;
    }
  }
  return true;
}

bool ScenarioCruiseDecider::CheckCenteringTriggered() {
  if (!non_motorway_config_.enable_bias_drive && near_obstacles_.empty() &&
      far_obstacles_.empty())
    return true;
  else
    return false;
}

bool ScenarioCruiseDecider::CheckCenteringFinished() {
  if (!non_motorway_config_.enable_bias_drive && near_obstacles_.empty() &&
      far_obstacles_.empty() && data_center_->is_auto_driving())
    return false;
  else
    return true;
}

bool ScenarioCruiseDecider::CheckBiasDriveTriggered() {
  if (non_motorway_config_.enable_bias_drive && near_obstacles_.empty() &&
      far_obstacles_.empty())
    return true;
  else
    return false;
}

bool ScenarioCruiseDecider::CheckBiasDriveFinished() {
  if (non_motorway_config_.enable_bias_drive && near_obstacles_.empty() &&
      far_obstacles_.empty() && data_center_->is_auto_driving())
    return false;
  else
    return true;
}

bool ScenarioCruiseDecider::CheckRightAvoidTriggered() {
  if (!cruise_scenario_conf_.enable_right_avoid) {
    return false;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) return false;
  const auto &outside_data = task_info.last_frame()->outside_planner_data();
  const auto &adc_boundary = outside_data.path_obstacle_context.adc_boundary;
  for (auto &obs : near_obstacles_) {
    if ((obs.min_l() < adc_boundary.start_l() ||
         obs.max_l() < adc_boundary.end_l())) {
      return false;
    }
  }
  if (near_obstacles_.empty() && only_front_slow_obs_) {
    return false;
  }
  return true;
}

bool ScenarioCruiseDecider::CheckRightAvoidFinished() {
  if (!cruise_scenario_conf_.enable_right_avoid) {
    return true;
  }
  if (!data_center_->is_auto_driving()) {
    LOG_INFO("It's in manual mode.");
    return true;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) return false;
  const auto &outside_data = task_info.last_frame()->outside_planner_data();
  const auto &adc_boundary = outside_data.path_obstacle_context.adc_boundary;

  for (auto &obs : near_obstacles_) {
    if ((obs.min_l() < adc_boundary.start_l() ||
         obs.max_l() < adc_boundary.end_l())) {
      return true;
    }
  }
  if (near_obstacles_.empty() && only_front_slow_obs_) {
    return true;
  }
  if (near_obstacles_.empty() && far_obstacles_.empty()) {
    return true;
  }
  return false;
}

bool ScenarioCruiseDecider::CheckLeftOvertakeTriggered() {
  if (!cruise_scenario_conf_.enable_left_overtake) {
    return false;
  }
  if (near_obstacles_.empty() && only_front_slow_obs_) {
    return true;
  }
  if (far_obstacles_.empty() && only_near_right_front_obs_) {
    return true;
  }
  return false;
}

bool ScenarioCruiseDecider::CheckLeftOvertakeFinished() {
  if (!cruise_scenario_conf_.enable_left_overtake) {
    return true;
  }
  if (!data_center_->is_auto_driving()) {
    LOG_INFO("It's in manual mode.");
    return true;
  }
  if (near_obstacles_.empty() && only_front_slow_obs_) {
    return false;
  }
  if (far_obstacles_.empty() && only_near_right_front_obs_) {
    return false;
  }
  return true;
}

bool ScenarioCruiseDecider::CheckPullOverTriggered() {
  auto &task_info = data_center_->task_info_list().front();
  double distance_to_end =
      data_center_->mutable_master_info()->distance_to_end();
  double vel = std::abs(data_center_->vehicle_state_proxy().LinearVelocity());
  bool need_pull_over = false;
  if (data_center_->routing_result().ptr->has_routing_request() &&
      data_center_->routing_result()
          .ptr->routing_request()
          .has_pull_over_at_destination()) {
    need_pull_over = data_center_->routing_result()
                         .ptr->routing_request()
                         .pull_over_at_destination();
  } else {
    need_pull_over = FLAGS_planning_enable_destination_pull_over;
  }
  if (!need_pull_over) return false;
  if (distance_to_end > FLAGS_planning_station_pull_over_look_forward_dis)
    return false;
  if (vel > 4.17) return false;
  if (!AllowPullOverInTurn(task_info)) return false;
  return true;
}

bool ScenarioCruiseDecider::CheckPullOverFinished() {
  double vel = std::abs(data_center_->vehicle_state_proxy().LinearVelocity());
  if (std::abs(data_center_->master_info().pull_over_distance_to_goal()) <
          5 * FLAGS_planning_arrive_to_destination_distance_threshold &&
      vel <= 2 * FLAGS_planning_adc_stop_velocity_threshold) {
    LOG_INFO("reach pull over goal.");
    return true;
  }
  if (!data_center_->is_auto_driving()) {
    LOG_INFO("It's in manual mode.");
    return true;
  }
  return false;
}

bool ScenarioCruiseDecider::AllowPullOverInTurn(const TaskInfo &task_info) {
  auto reference_line = task_info.reference_line();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  // check curvature, right(-), left(+)
  for (const auto &pt : reference_line->ref_points()) {
    if (pt.s() >
        task_info.curr_sl().s() + data_center_->master_info().distance_to_end())
      break;
    if (pt.s() <= task_info.curr_sl().s()) continue;
    if (FLAGS_planning_default_left_right_side) {
      if (pt.kappa() > plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has left turn.");
        return true;
      } else if (pt.kappa() <
                 -plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has right turn.");
        return false;
      }
    } else {
      if (pt.kappa() > plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has left turn.");
        return false;
      } else if (pt.kappa() <
                 -plan_config.intention.turn_intention_kappa_threshold) {
        LOG_INFO("front lane has right turn.");
        return true;
      }
    }
  }
  return true;
}

void ScenarioCruiseDecider::SaveMonitorMessage() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  static char str_buffer[256];
  sprintf(str_buffer,
          "[CRUISE][near obs: %d] [far obs: %d] [target l: %f] "
          "[only_front_slow_obs: %d] [only_near_right_front_obs: %d] [filter "
          "veh_v: %f] [is_front_turn: %d] [is_in_junction: %d]",
          near_obstacles_.size(), far_obstacles_.size(), observe_ref_l,
          only_front_slow_obs_, only_near_right_front_obs_, veh_v_,
          is_front_turn_, is_in_junction_);
  DataCenter::Instance()->SetMonitorString(str_buffer,
                                           MonitorItemSource::CRUISE_STATE);
}

}  // namespace planning
}  // namespace neodrive
