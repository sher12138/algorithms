#include "scenario_motorway_lane_change_decider.h"

namespace neodrive {
namespace planning {
namespace {
double GetStopSpeedLimit(const double dist_gap, const double deaccel) {
  return dist_gap > 0 ? std::sqrt(2.0 * dist_gap * deaccel) : 0.0;
}
}  // namespace

constexpr int ScenarioMotorwayLaneChangeDecider::kFilterFrameThreshold;

ScenarioMotorwayLaneChangeDecider::ScenarioMotorwayLaneChangeDecider() {
  name_ = "ScenarioMotorwayLaneChangeDecider";
}

bool ScenarioMotorwayLaneChangeDecider::Init() {
  if (initialized_) {
    return true;
  }

  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/"
      "scenario_motorway_lane_change_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  plan_config_ptr_ = &config::PlanningConfig::Instance()->plan_config();

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioMotorwayLaneChangeDecider init.");
  return true;
}

bool ScenarioMotorwayLaneChangeDecider::Reset() {
  if (!ResetStateMachine()) return false;
  LOG_INFO("Reset ScenarioMotorwayLaneChangeDecider.");
  return true;
}

ErrorCode ScenarioMotorwayLaneChangeDecider::RunOnce() {
  LOG_INFO("ScenarioMotorwayLaneChangeDecider::RunOnce");

  is_state_change_ = false;
  GetTargetL();
  ComputeEndDis();
  ComputeRemainTime();
  PrepareMotorwayLaneChangeCondition();
  ExtendLaneBound();
  // CreateLaneChangeVirtualObs();
  auto it = state_machine_function_map_.find(curr_state());
  if (it != state_machine_function_map_.end()) {
    auto func = it->second;
    (this->*func)();
  } else {
    // missing handle state function
    LOG_ERROR("missing handle state function {}", curr_state_str_);
  }
  if (is_state_change_) {
    if (!UpdateStateMachine()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  LimitRefLByObs();
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::RefLFilter(task_info);
  SaveMonitorMessage();
  return ErrorCode::PLANNING_OK;
}

bool ScenarioMotorwayLaneChangeDecider::ResetStateMachine() {
  if (!state_machine_.SetInitializeState("PREPARE")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(static_cast<MotorwayLaneChangeStageState::State>(
      state_machine_.GetCurrentState()));
  prev_decision_ = MotorwayLaneChangeStageChangeFlag::T_NONE_PREPARE;
  curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_NONE_PREPARE;
  return true;
}

void ScenarioMotorwayLaneChangeDecider::
    RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<MotorwayLaneChangeStageState::State,
                void (ScenarioMotorwayLaneChangeDecider::*)()>(
          MotorwayLaneChangeStageState::PREPARE,
          &ScenarioMotorwayLaneChangeDecider::OnHandleStatePrepare));
  state_machine_function_map_.insert(
      std::pair<MotorwayLaneChangeStageState::State,
                void (ScenarioMotorwayLaneChangeDecider::*)()>(
          MotorwayLaneChangeStageState::WAITING,
          &ScenarioMotorwayLaneChangeDecider::OnHandleStateWaiting));
  state_machine_function_map_.insert(
      std::pair<MotorwayLaneChangeStageState::State,
                void (ScenarioMotorwayLaneChangeDecider::*)()>(
          MotorwayLaneChangeStageState::CHANGING,
          &ScenarioMotorwayLaneChangeDecider::OnHandleStateChanging));
  state_machine_function_map_.insert(
      std::pair<MotorwayLaneChangeStageState::State,
                void (ScenarioMotorwayLaneChangeDecider::*)()>(
          MotorwayLaneChangeStageState::FINISH,
          &ScenarioMotorwayLaneChangeDecider::OnHandleStateFinish));
  state_machine_function_map_.insert(
      std::pair<MotorwayLaneChangeStageState::State,
                void (ScenarioMotorwayLaneChangeDecider::*)()>(
          MotorwayLaneChangeStageState::CANCEL,
          &ScenarioMotorwayLaneChangeDecider::OnHandleStateCancel));
}

bool ScenarioMotorwayLaneChangeDecider::UpdateStateMachine() {
  // filter decision.
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  }
  // Temporarily, all states are not considered for filtering
  else if (curr_state() != MotorwayLaneChangeStageState::WAITING ||
           curr_decision_ ==
               MotorwayLaneChangeStageChangeFlag::T_WAITING_CANCEL ||
           curr_decision_ ==
               MotorwayLaneChangeStageChangeFlag::T_CHANGING_CANCEL ||
           curr_decision_ ==
               MotorwayLaneChangeStageChangeFlag::T_PREPARE_CANCEL) {
    filter_decision_ = curr_decision_;
    decision_filter_count = kFilterFrameThreshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO(
        "filter decision: {}",
        MotorwayLaneChangeStageChangeFlag::ChangeFlag_Name(curr_decision_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state.
    LOG_INFO(
        "hold stage: {}, decision_filter_count: {}, kFilterFrameThreshold: {}",
        curr_state(), decision_filter_count, kFilterFrameThreshold);
    return true;
  }
  // update state.
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR(
        "{}: state change fail. state: {}, changeflag: {}", name_,
        curr_state_str_,
        MotorwayLaneChangeStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(static_cast<MotorwayLaneChangeStageState::State>(
        state_machine_.GetCurrentState()));
    prev_decision_ = curr_decision_;
    if (curr_state_str_ != prev_state) {
      prev_state_str_ = prev_state;
      LOG_INFO("{}: update motorway_lane_change stage: ({}) {} {}", Name(),
               state_machine_.GetChangeFlagStr(curr_decision_), prev_state,
               curr_state_str_);
    }
  }
  return true;
}

void ScenarioMotorwayLaneChangeDecider::PrepareMotorwayLaneChangeCondition() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  motorway_lane_change_context->is_adc_on_current_lane = IsOnCurrentLane();
  // motorway_lane_change_context->is_adc_on_target_lane = IsOnTargetLane();
  motorway_lane_change_context->is_adc_on_target_lane =
      IsOverCurrentLane() || IsOnTargetLane();
  motorway_lane_change_context->is_adc_over_current_lane = IsOverCurrentLane();
  motorway_lane_change_context->is_target_lane_dynamic_obs_clear =
      IsTargetLaneDynamicObsClear();
  motorway_lane_change_context->is_target_lane_static_obs_clear =
      IsTargetLaneStaticObsClear();
  motorway_lane_change_context->is_front_has_traffic_light =
      IsFrontHasTrafficLight();
  motorway_lane_change_context->is_front_has_road_boundary =
      IsFrontHasRoadBoundary();
  motorway_lane_change_context->is_front_has_divider_restrict =
      IsFrontHasDividerRestrict();
  motorway_lane_change_context->is_front_has_lane_turn = IsFrontHasLaneTurn();
}

bool ScenarioMotorwayLaneChangeDecider::IsFrontHasTrafficLight() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &signals = task_info.reference_line()->signal_overlaps();
  if (signals.empty()) {
    LOG_INFO("front has not signals.");
    return false;
  }
  auto &preview_distance = plan_config_ptr_->motorway_lane_change_scenario
                               .traffic_light_preview_distance;
  for (const auto &signal : signals) {
    if (task_info.curr_sl().s() + 0.0 > signal.end_s ||
        task_info.curr_sl().s() + preview_distance < signal.start_s) {
      continue;
    }
    LOG_INFO(
        "front has signal, cannot lane change. signal id: {}, start_s: {:.4f}, "
        "curr_s: {:.4f}",
        signal.object_id, signal.start_s, task_info.curr_sl().s());
    return true;
  }
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::IsFrontHasLaneTurn() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &ref_points = task_info.reference_line()->ref_points();
  auto &preview_distance = plan_config_ptr_->motorway_lane_change_scenario
                               .lane_turn_preview_distance;
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    if (ref_points[i].s() < task_info.curr_sl().s() + 0.0) {
      continue;
    }
    if (ref_points[i].s() > task_info.curr_sl().s() + preview_distance) {
      break;
    }
    if (ref_points[i].is_uturn_signal() || ref_points[i].is_right_signal() ||
        ref_points[i].is_left_signal()) {
      LOG_INFO("front has lane turn, could not lane change.");
      return true;
    }
  }
  LOG_INFO("front has not lane turn.");
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::IsFrontHasRoadBoundary() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &ref_points = task_info.reference_line()->ref_points();
  auto &preview_distance = plan_config_ptr_->motorway_lane_change_scenario
                               .road_bound_preview_distance;
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + 0.0) {
      continue;
    }
    auto navigator_request =
        data_center_->navigation_result().navigator_request;
    if (navigator_request == REQUEST_LEFT_LANE_CHANGE &&
        std::fabs(ref_points[i].left_road_bound() -
                  ref_points[i].left_lane_bound()) < 0.3) {
      LOG_INFO("left front has road boundary.");
      return true;
    }
    if (navigator_request == REQUEST_RIGHT_LANE_CHANGE &&
        std::fabs(ref_points[i].right_road_bound() -
                  ref_points[i].right_lane_bound()) < 0.3) {
      LOG_INFO("right front has road boundary.");
      return true;
    }
    if (ref_points[i].s() > preview_distance + task_info.curr_sl().s()) {
      break;
    }
  }
  LOG_INFO("front has not road boundary.");
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::IsFrontHasDividerRestrict() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &ref_points = task_info.reference_line()->ref_points();
  auto &preview_distance = plan_config_ptr_->motorway_lane_change_scenario
                               .divider_restrict_preview_distance;
  for (std::size_t i = task_info.referline_curr_index(); i < ref_points.size();
       ++i) {
    // ignore too close.
    if (ref_points[i].s() < task_info.curr_sl().s() + 0.0) {
      continue;
    }

    auto navigator_request =
        data_center_->navigation_result().navigator_request;

    // LONG_DASHED_LINE & WHITE = DOTTED_WHITE
    // SINGLE_SOLID_LINE & WHITE = SOLID_WHITE
    if (navigator_request == REQUEST_LEFT_LANE_CHANGE) {
      std::vector<DividerFeature> left_divider_feature =
          ref_points[i].left_divider_feature();
      int cross_cnt = 0;
      for (int i = 0; i < left_divider_feature.size(); ++i) {
        DividerFeature divider_feature = left_divider_feature[i];
        if (divider_feature.divider_type_ ==
                DividerFeature::DividerType::LONG_DASHED_LINE &&
            divider_feature.divider_color_ ==
                DividerFeature::DividerColor::WHITE)
          cross_cnt++;
      }
      if (cross_cnt != left_divider_feature.size()) {
        LOG_INFO("left front has divider restrict.");
        return true;
      }
    }

    if (navigator_request == REQUEST_RIGHT_LANE_CHANGE) {
      std::vector<DividerFeature> right_divider_feature =
          ref_points[i].right_divider_feature();
      int cross_cnt = 0;
      for (int i = 0; i < right_divider_feature.size(); ++i) {
        DividerFeature divider_feature = right_divider_feature[i];
        if (divider_feature.divider_type_ ==
                DividerFeature::DividerType::LONG_DASHED_LINE &&
            divider_feature.divider_color_ ==
                DividerFeature::DividerColor::WHITE)
          cross_cnt++;
      }
      if (cross_cnt != right_divider_feature.size()) {
        LOG_INFO("right front has divider restrict.");
        return true;
      }
    }

    if (ref_points[i].s() > preview_distance + task_info.curr_sl().s()) {
      break;
    }
  }
  LOG_INFO("front has not divider restrict.");
  return false;
}

void ScenarioMotorwayLaneChangeDecider::CreateLaneChangeVirtualObs() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto decision_data =
      task_info.last_frame()->planning_data().mutable_decision_data();
  if (decision_data == nullptr) {
    LOG_INFO("decision_data null");
    return;
  }

  // process
  if (decision_data->create_virtual_obstacle(
          Vec2d{change_end_point_.x(), change_end_point_.y()}, 10.0,
          FLAGS_planning_virtual_obstacle_height,
          change_end_point_.left_lane_bound() +
              change_end_point_.right_lane_bound(),
          change_end_point_.heading(),
          VirtualObstacle::LANECHANGE) != ErrorCode::PLANNING_OK) {
    LOG_ERROR("Failed to create lane change virtual_obstacle.");
    return;
  }
}

bool ScenarioMotorwayLaneChangeDecider::ExtendLaneBound() {
  auto &task_info = data_center_->task_info_list().front();
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  auto &reference_line = task_info.reference_line();
  auto target_reference_line = data_center_->target_odom_ref();
  auto navigator_request = data_center_->navigation_result().navigator_request;

  ReferencePoint reference_point;
  for (std::size_t i = 0; i < reference_line->ref_points().size(); ++i) {
    const auto &ref_pt = reference_line->ref_points().at(i);
    if (ref_pt.s() > inside_data.init_sl_point.s() + kExtendFrontDis) break;
    if (ref_pt.s() < inside_data.init_sl_point.s() - kExtendBackDis) continue;
    target_reference_line->GetNearestRefPoint(Vec2d{ref_pt.x(), ref_pt.y()},
                                              &reference_point);
    SLPoint target_point;
    reference_line->GetPointInFrenetFrame(
        Vec2d{reference_point.x(), reference_point.y()}, &target_point);

    if (navigator_request == REQUEST_LEFT_LANE_CHANGE) {
      task_info.reference_line()->SetIndexBound(
          i,
          std::min(target_point.l() + reference_point.left_lane_bound(),
                   ref_pt.left_road_bound()),
          ref_pt.right_lane_bound());
      task_info.reference_line()->SetIndexLaneBorrowFlag(i, true, false);
    }
    if (navigator_request == REQUEST_RIGHT_LANE_CHANGE) {
      task_info.reference_line()->SetIndexBound(
          i, ref_pt.left_lane_bound(),
          std::min(-target_point.l() + reference_point.right_lane_bound(),
                   ref_pt.right_road_bound()));
      task_info.reference_line()->SetIndexLaneBorrowFlag(i, false, true);
    }
  }
  return true;
}

bool ScenarioMotorwayLaneChangeDecider::GetTargetL() {
  auto &task_info = data_center_->task_info_list().front();
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  auto &reference_line = task_info.reference_line();
  auto target_reference_line = data_center_->target_odom_ref();
  ReferencePoint reference_point;
  target_reference_line->GetNearestRefPoint(
      Vec2d{inside_data.init_point.x(), inside_data.init_point.y()},
      &reference_point);
  reference_line->GetPointInFrenetFrame(
      Vec2d{reference_point.x(), reference_point.y()}, &target_point_);
  LOG_INFO(
      "init_point x: {:.4f}, y: {:.4f}, reference_point x: {:.4f}, y: {:.4f}, "
      "target l: {:.4f}",
      inside_data.init_point.x(), inside_data.init_point.y(),
      reference_point.x(), reference_point.y(), target_point_.l());
  return true;
}

void ScenarioMotorwayLaneChangeDecider::LimitRefLByObs() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  if (task_info.last_frame() == nullptr) return;
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  double preview_distance =
      std::max(FLAGS_planning_trajectory_min_length,
               task_info.last_frame()->inside_planner_data().vel_v * 5.0);
  auto &shrink_bounds_info =
      task_info.last_frame()->outside_planner_data().shrink_half_ego_boundaries;

  if (shrink_bounds_info.empty()) return;
  auto right_most_boundry_point = shrink_bounds_info.front();
  auto left_most_boundry_point = shrink_bounds_info.front();
  for (auto &point : shrink_bounds_info) {
    if (point.lower_point.s() >
        task_info.last_frame()->inside_planner_data().init_sl_point.s() +
            preview_distance) {
      break;
    }
    if (point.lower_type == PathRegion::Bound::BoundType::OBS &&
        right_most_boundry_point.lower_type !=
            PathRegion::Bound::BoundType::OBS) {
      right_most_boundry_point = point;
    } else if (point.lower_type == PathRegion::Bound::BoundType::OBS &&
               point.lower_point.l() >
                   right_most_boundry_point.lower_point.l()) {
      right_most_boundry_point = point;
    }
    if (point.upper_type == PathRegion::Bound::BoundType::OBS &&
        left_most_boundry_point.upper_type !=
            PathRegion::Bound::BoundType::OBS) {
      left_most_boundry_point = point;
    } else if (point.upper_type == PathRegion::Bound::BoundType::OBS &&
               point.upper_point.l() >
                   left_most_boundry_point.upper_point.l()) {
      left_most_boundry_point = point;
    }
  }

  double lower_bound = right_most_boundry_point.lower_point.l();
  double upper_bound = left_most_boundry_point.upper_point.l();
  const double bias_to_obs_bound =
      plan_config_ptr_->common.motorway_bias_to_obs_bound;
  // compute ref_l when enable bias drive
  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      if (left_most_boundry_point.upper_type ==
          PathRegion::Bound::BoundType::OBS) {
        observe_ref_l = std::min(
            left_most_boundry_point.upper_point.l() - bias_to_obs_bound,
            observe_ref_l);
      }
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      if (right_most_boundry_point.lower_type ==
          PathRegion::Bound::BoundType::OBS) {
        observe_ref_l = std::max(
            right_most_boundry_point.lower_point.l() + bias_to_obs_bound,
            observe_ref_l);
      }
      break;
    default:
      break;
  }
  LOG_INFO(
      "bias_to_obs_bound: {:.4f}, right bound l: {:.4f}, left bound l: {:.4f}, "
      "observe_ref_l: {:.4f}",
      bias_to_obs_bound, lower_bound, upper_bound, observe_ref_l);
}

bool ScenarioMotorwayLaneChangeDecider::IsOnCurrentLane() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &adc_boundary = task_info.last_frame()
                                 ->outside_planner_data()
                                 .path_obstacle_context.adc_boundary;
  return ((adc_boundary.end_l() <
           task_info.curr_referline_pt().left_lane_bound()) &&
          (adc_boundary.start_l() >
           -task_info.curr_referline_pt().right_lane_bound()));
}

bool ScenarioMotorwayLaneChangeDecider::IsOverCurrentLane() const {
  auto &task_info = data_center_->task_info_list().front();
  auto &adc_boundary = task_info.adc_boundary();
  auto &adc_width_ratio =
      plan_config_ptr_->motorway_lane_change_scenario.adc_width_ratio;
  LOG_INFO("adc start l: {:.4f}, end l: {:.4f}, adc_width_ratio: {:.4f}",
           adc_boundary.start_l(), adc_boundary.end_l(), adc_width_ratio);
  double adc_ratio_l{0.0};
  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      adc_ratio_l = adc_boundary.end_l() -
                    adc_width_ratio *
                        std::abs(adc_boundary.end_l() - adc_boundary.start_l());
      LOG_INFO("adc_ratio_l: {:.4f}, bound l: {:.4f}", adc_ratio_l,
               task_info.curr_referline_pt().left_lane_bound());
      return adc_ratio_l > task_info.curr_referline_pt().left_lane_bound();
    case REQUEST_RIGHT_LANE_CHANGE:
      adc_ratio_l = adc_boundary.start_l() +
                    adc_width_ratio *
                        std::abs(adc_boundary.end_l() - adc_boundary.start_l());
      LOG_INFO("adc_ratio_l: {:.4f}, bound l: {:.4f}", adc_ratio_l,
               -task_info.curr_referline_pt().right_lane_bound());
      return adc_ratio_l < -task_info.curr_referline_pt().right_lane_bound();
    default:
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      return false;
  }
}

bool ScenarioMotorwayLaneChangeDecider::IsOnTargetLane() const {
  auto &task_info = data_center_->task_info_list().front();
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  auto target_reference_line = data_center_->target_odom_ref();
  neodrive::planning::Boundary adc_boundary{};
  BuildAdcBoundary(target_reference_line, inside_data.init_point, adc_boundary);
  ReferencePoint reference_point;
  if (!target_reference_line->GetNearestRefPoint(
          Vec2d{inside_data.init_point.x(), inside_data.init_point.y()},
          &reference_point)) {
    LOG_INFO("GetNearestRefPoint fail");
    return false;
  }
  auto &adc_width_ratio =
      plan_config_ptr_->motorway_lane_change_scenario.adc_width_ratio;
  LOG_INFO("adc start l: {:.4f}, end l: {:.4f}, adc_width_ratio: {:.4f}",
           adc_boundary.start_l(), adc_boundary.end_l(), adc_width_ratio);
  double adc_ratio_l{0.0};
  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      adc_ratio_l = adc_boundary.end_l() -
                    adc_width_ratio *
                        std::abs(adc_boundary.end_l() - adc_boundary.start_l());
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      adc_ratio_l = adc_boundary.start_l() +
                    adc_width_ratio *
                        std::abs(adc_boundary.end_l() - adc_boundary.start_l());
      break;
    default:
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      return false;
  }
  LOG_INFO(
      "adc_ratio_l: {:.4f}, left_lane_bound: {:.4f}, right_lane_bound: {:.4f}",
      adc_ratio_l, reference_point.left_lane_bound(),
      reference_point.right_lane_bound());
  return (adc_ratio_l < reference_point.left_lane_bound() &&
          adc_ratio_l > -reference_point.right_lane_bound());
}

void ScenarioMotorwayLaneChangeDecider::ComputeEndDis() {
  auto &task_info = data_center_->mutable_task_info_list()->front();

  dis_to_current_ref_end_ =
      task_info.reference_line()->ref_points().back().s() -
      task_info.last_frame()->inside_planner_data().init_sl_point.s() -
      plan_config_ptr_->motorway_lane_change_scenario
          .min_stop_dist_before_junction;
  LOG_INFO("dis_to_current_ref_end_:{:.4f}, cur s:{:.4f}",
           dis_to_current_ref_end_,
           task_info.last_frame()->inside_planner_data().init_sl_point.s());

  auto &lane_change_end_point =
      data_center_->navigation_result().lane_change_end_point;
  if (task_info.reference_line_raw()->GetNearestRefPoint(
          Vec2d{lane_change_end_point.x(), lane_change_end_point.y()},
          &change_end_point_)) {
    // utm to odom
    task_info.reference_line()->GetNearestRefPoint(change_end_point_.s(),
                                                   &change_end_point_);
    dis_to_change_end_ = change_end_point_.s() - task_info.curr_sl().s();
  } else {
    dis_to_change_end_ =
        data_center_->navigation_result().dist_to_lane_change_end;
  }

  LOG_INFO(
      "dis_to_change_end: {:.4f}, change_end_point s: {:.4f}, curr_s: {:.4f}",
      dis_to_change_end_, change_end_point_.s(), task_info.curr_sl().s());

  if (dis_to_change_end_ < -kMathEpsilon ||
      (dis_to_change_end_ < plan_config_ptr_->motorway_lane_change_scenario
                                .min_distance_to_lane_change_end &&
       data_center_->vehicle_state_odometry().LinearVelocity() < 0.1)) {
    LOG_INFO("set lane change failed");
    data_center_->mutable_event_report_proxy()->SetEvent(
        EventType::LANE_CHANGE_FAIL);
  } else {
    data_center_->mutable_event_report_proxy()->EventReset(
        EventType::LANE_CHANGE_FAIL);
    LOG_INFO("reset lane change failed");
  }
}

void ScenarioMotorwayLaneChangeDecider::ComputeRemainTime() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &lanechange_conf = plan_config_ptr_->motorway_lane_change_scenario;
  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  auto &k = ref_conf.vhe_v_filter_ratio;
  veh_v_ = veh_v_ * (1.0 - k) +
           k * task_info.last_frame()->inside_planner_data().vel_v;

  remain_lat_dis_ = target_point_.l() - task_info.curr_sl().l();

  double vel_heading_diff =
      task_info.last_frame()->inside_planner_data().vel_heading -
      task_info.curr_referline_pt().heading();

  double v_s = veh_v_ * std::cos(vel_heading_diff);
  double v_l = veh_v_ * std::sin(vel_heading_diff);

  double preview_time = remain_lat_dis_ / v_l;
  double lanechange_max_time =
      lanechange_conf.preview_time * std::abs(remain_lat_dis_) / 3.5;
  remain_time_ = preview_time > 0 ? std::min(preview_time, lanechange_max_time)
                                  : lanechange_max_time;
  change_need_dis_ = veh_v_ * remain_time_;
  LOG_INFO(
      "remain_lat_dis: {:.4f}, vel_heading_diff: {:.4f}, vel_heading: {:.4f}, "
      "referline heading: {:.4f}, veh_v_: {:.4f}, v_s: {:.4f}, v_l: {:.4f}, "
      "remain_time: {:.4f}, : {:.4f}, lanechange_max_time: {:.4f}, "
      "change_need_dis: {:.4f}",
      remain_lat_dis_, vel_heading_diff * 57.3,
      task_info.last_frame()->inside_planner_data().vel_heading * 57.3,
      task_info.curr_referline_pt().heading() * 57.3, veh_v_, v_s, v_l,
      remain_time_, preview_time, lanechange_max_time, change_need_dis_);
}

bool ScenarioMotorwayLaneChangeDecider::IsTargetLaneStaticObsClear() {
  auto &task_info = data_center_->task_info_list().front();
  const auto &inside_data = task_info.last_frame()->inside_planner_data();

  ReferencePoint reference_point;
  auto target_reference_line = data_center_->target_odom_ref();
  target_reference_line->GetNearestRefPoint(
      Vec2d{inside_data.init_point.x(), inside_data.init_point.y()},
      &reference_point);

  double left_lane_bound = task_info.curr_referline_pt().left_lane_bound();
  double right_lane_bound = task_info.curr_referline_pt().right_lane_bound();

  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      right_lane_bound = task_info.curr_referline_pt().left_lane_bound();
      left_lane_bound = task_info.curr_referline_pt().left_lane_bound() +
                        reference_point.left_lane_bound() +
                        reference_point.right_lane_bound();
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      left_lane_bound = -task_info.curr_referline_pt().right_lane_bound();
      right_lane_bound = -task_info.curr_referline_pt().right_lane_bound() -
                         (reference_point.left_lane_bound() +
                          reference_point.right_lane_bound());
      break;
    default:
      break;
  }

  double preview_s = task_info.curr_sl().s() + change_need_dis_;
  std::vector<Obstacle *> static_obs{};
  int obs_id = -1;
  double obs_end_s = 0.0;
  bool isclear = scenario_common::IsFrontStaticObsClear(
      task_info, left_lane_bound, right_lane_bound, preview_s, obs_end_s,
      obs_id, static_obs);
  LOG_INFO(
      "cur bound: {:.4f}/{:.4f}, target bound: {:.4f}/{:.4f}, search bound: "
      "{:.4f}/{:.4f}, obs_end_s: {:.4f}",
      task_info.curr_referline_pt().left_lane_bound(),
      task_info.curr_referline_pt().right_lane_bound(),
      reference_point.left_lane_bound(), reference_point.right_lane_bound(),
      left_lane_bound, right_lane_bound, obs_end_s);
  return isclear;
}

bool ScenarioMotorwayLaneChangeDecider::IsTargetLaneDynamicObsClear() {
  near_obstacles_.clear();
  far_obstacles_.clear();
  // prepare data
  auto &task_info = data_center_->task_info_list().front();
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  const auto &decision_data = task_info.decision_data();
  auto &reference_line = task_info.reference_line();
  auto target_reference_line = data_center_->target_odom_ref();
  neodrive::planning::Boundary adc_boundary{};
  BuildAdcBoundary(target_reference_line, inside_data.init_point, adc_boundary);

  auto &ref_conf = config::PlanningConfig::Instance()
                       ->planning_research_config()
                       .path_observe_ref_decider_config;
  auto &lanechange_conf = plan_config_ptr_->motorway_lane_change_scenario;
  auto &preview_front_distance = lanechange_conf.preview_front_distance;
  auto &preview_back_distance = lanechange_conf.preview_back_distance;
  auto &near_front_distance = lanechange_conf.near_front_distance;
  auto &near_back_distance = lanechange_conf.near_back_distance;

  LOG_INFO(
      "adc_boundary.start_s: {:.4f}, end_s: {:.4f}, start_l: "
      "{:.4f}, end_l: {:.4f}, v: {:.4f}",
      adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
      adc_boundary.end_l(), veh_v_);
  for (size_t i = 0; i < decision_data->dynamic_obstacle().size(); ++i) {
    auto obstacle = *decision_data->dynamic_obstacle()[i];
    obstacle.init_with_reference_line(target_reference_line);
    LOG_INFO(
        "obstacle[{}] min_s: {:.4f}, max_s: {:.4f}, min_l: {:.4f}, max_l: "
        "{:.4f}, v: {:.4f}",
        obstacle.id(), obstacle.min_s(), obstacle.max_s(), obstacle.min_l(),
        obstacle.max_l(), obstacle.speed());

    if (obstacle.max_s() < adc_boundary.end_s() - preview_back_distance ||
        obstacle.min_s() > adc_boundary.start_s() + preview_front_distance) {
      LOG_INFO("obs too far. max s: {:.4f}, min s: {:.4f}", obstacle.max_s(),
               obstacle.min_s());
      continue;
    }
    ReferencePoint reference_point;
    target_reference_line->GetNearestRefPoint(obstacle.center_sl().s(),
                                              &reference_point);
    if (obstacle.center_sl().l() < -reference_point.right_lane_bound() ||
        obstacle.center_sl().l() > reference_point.left_lane_bound()) {
      LOG_INFO(
          "obs not in target lane bound. center l: {:.4f}, left "
          "bound: {:.4f}, right bound: {:.4f}",
          obstacle.center_sl().l(), reference_point.left_lane_bound(),
          reference_point.right_lane_bound());
      continue;
    }

    double heading_diff = std::abs(normalize_angle(obstacle.velocity_heading() -
                                                   reference_point.heading()));
    if (heading_diff > M_PI / 2.0 &&
        obstacle.max_s() < adc_boundary.start_s()) {
      LOG_INFO("skip obverse back obstacle[{}] heading_diff: {:.4f}",
               obstacle.id(), heading_diff);
      continue;
    }
    if ((obstacle.max_s() > adc_boundary.start_s() - near_back_distance &&
         obstacle.max_s() < adc_boundary.end_s() + near_front_distance) ||
        (obstacle.min_s() > adc_boundary.start_s() - near_back_distance &&
         obstacle.min_s() < adc_boundary.end_s() + near_front_distance)) {
      near_obstacles_.push_back(obstacle);
      LOG_INFO("find near obs[{}]", obstacle.id());
      continue;
    }

    if (heading_diff > ref_conf.filter_obs_heading_threshold &&
        M_PI - heading_diff > ref_conf.filter_obs_heading_threshold) {
      LOG_INFO("skip oblique obstacle[{}] heading_diff: {:.4f}", obstacle.id(),
               heading_diff);
      continue;
    }

    if (heading_diff < M_PI / 2.0) {
      if (obstacle.min_s() > adc_boundary.end_s() + near_front_distance &&
          obstacle.speed() > veh_v_) {
        LOG_INFO(
            "skip front high speed obs[{}]: obs speed: {:.4f}, veh_v_: "
            "{:.4f}",
            obstacle.id(), obstacle.speed(), veh_v_);
        continue;
      }
      if (obstacle.max_s() < adc_boundary.start_s() - near_back_distance &&
          obstacle.speed() < veh_v_) {
        LOG_INFO(
            "skip back low speed obs[{}]: obs speed: {:.4f}, veh_v_: {:.4f}",
            obstacle.id(), obstacle.speed(), veh_v_);
        continue;
      }
      if (obstacle.min_s() > adc_boundary.end_s() + near_front_distance &&
          obstacle.speed() < veh_v_) {
        double delt_s =
            obstacle.min_s() - near_front_distance - adc_boundary.end_s();
        double delt_v = veh_v_ - obstacle.speed();
        double delt_t = delt_s / delt_v;
        if (delt_t < remain_time_ / 2.0) {
          LOG_INFO(
              "find front obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          far_obstacles_.push_back(obstacle);
        } else {
          LOG_INFO(
              "skip front obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }
      }
      if (obstacle.max_s() < adc_boundary.start_s() &&
          obstacle.speed() > veh_v_) {
        double delt_s = adc_boundary.start_s() - obstacle.max_s();
        double delt_v = obstacle.speed() - veh_v_;
        double delt_t = delt_s / delt_v;
        if (delt_t < remain_time_) {
          LOG_INFO(
              "find back obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          far_obstacles_.push_back(obstacle);
        } else {
          LOG_INFO(
              "skip back obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, delt_t: "
              "{:.4f}",
              obstacle.id(), delt_s, delt_v, delt_t);
          continue;
        }
      }
    } else {
      // Opposite direction
      if (obstacle.max_s() < adc_boundary.start_s()) {
        LOG_INFO("reverse back obs[{}]: obs max_s: {:.4f}, veh start_s: {:.4f}",
                 obstacle.id(), obstacle.max_s(), adc_boundary.start_s());
        continue;
      }
      double delt_s =
          obstacle.min_s() - adc_boundary.end_s() - near_front_distance;
      double delt_v = obstacle.speed() + veh_v_;
      double delt_t = delt_s / delt_v;
      if (delt_t < remain_time_) {
        LOG_INFO(
            "find front reverse obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        far_obstacles_.push_back(obstacle);
      } else {
        LOG_INFO(
            "skip front reverse obs[{}]: delt_s: {:.4f}, delt_v: {:.4f}, "
            "delt_t: {:.4f}",
            obstacle.id(), delt_s, delt_v, delt_t);
        continue;
      }
    }
  }
  if (near_obstacles_.empty() && far_obstacles_.empty()) {
    LOG_INFO("target lane is clear.");
    return true;
  } else {
    LOG_INFO(
        "target lane is not clear. near_obstacles size: {}, far_obstacles "
        "size: {}",
        near_obstacles_.size(), far_obstacles_.size());
    return false;
  }
}

void ScenarioMotorwayLaneChangeDecider::BuildAdcBoundary(
    const ReferenceLinePtr &reference_line, const TrajectoryPoint &init_point,
    Boundary &adc_boundary) const {
  Box2d adc_bounding_box = VehicleParam::Instance()->get_adc_bounding_box(
      {init_point.x(), init_point.y()}, init_point.theta(), 0.2, 0.2, 0.2);
  std::vector<Vec2d> points;
  adc_bounding_box.get_all_corners(&points);
  SLPoint sl_point;
  for (const auto &pt : points) {
    reference_line->GetPointInFrenetFrame(pt, &sl_point);
    adc_boundary.set_start_s(std::min(adc_boundary.start_s(), sl_point.s()));
    adc_boundary.set_end_s(std::max(adc_boundary.end_s(), sl_point.s()));
    adc_boundary.set_start_l(std::min(adc_boundary.start_l(), sl_point.l()));
    adc_boundary.set_end_l(std::max(adc_boundary.end_l(), sl_point.l()));
  }
}

void ScenarioMotorwayLaneChangeDecider::OnHandleStatePrepare() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  auto &inside_data = data_center_->last_frame()->inside_planner_data();

  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      observe_ref_l =
          std::min(inside_data.init_sl_point.l() + 1.0, target_point_.l());
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      observe_ref_l =
          std::max(inside_data.init_sl_point.l() - 1.0, target_point_.l());
      break;
    default:
      observe_ref_l = 0.0;
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      break;
  }

  if (CheckFinishTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_PREPARE_FINISH;
    return;
  }
  if (CheckCancelTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_PREPARE_CANCEL;
    return;
  }
  if (CheckWaitingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_PREPARE_WAITING;
    return;
  }
  if (CheckChangingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_PREPARE_CHANGING;
    return;
  }
}

void ScenarioMotorwayLaneChangeDecider::OnHandleStateWaiting() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  const auto &inside_data = task_info.last_frame()->inside_planner_data();
  double width_threshold = VehicleParam::Instance()->width() * 0.5;

  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("target_lane is not clear but adc is on lane.");
        observe_ref_l = task_info.curr_sl().l();
      } else {
        observe_ref_l =
            std::min(task_info.curr_sl().l() + 0.5,
                     task_info.curr_referline_pt().left_lane_bound() -
                         width_threshold - 0.3);
      }
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("target_lane is not clear but adc is on lane.");
        observe_ref_l = task_info.curr_sl().l();
      } else {
        observe_ref_l =
            std::max(task_info.curr_sl().l() - 0.5,
                     -task_info.curr_referline_pt().right_lane_bound() +
                         width_threshold + 0.3);
      }
      break;
    default:
      observe_ref_l = 0.0;
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      break;
  }

  double ref_end_speed_limit = GetStopSpeedLimit(
      dis_to_current_ref_end_,
      plan_config_ptr_->speed_limit.speed_limit_slow_down_accel);
  LOG_INFO(
      "ref_end_speed_limit: {:.4f}, dis_to_current_ref_end: {:.4f}, a: {:.4f}",
      ref_end_speed_limit, dis_to_current_ref_end_,
      plan_config_ptr_->speed_limit.speed_limit_slow_down_accel);

  double dis_to_change_end =
      motorway_lane_change_context->is_target_lane_static_obs_clear
          ? dis_to_change_end_ - 15.0
          : dis_to_change_end_;
  double change_speed_limit = std::max(dis_to_change_end / 10.0, 0.0);
  double speed_limit = std::min(ref_end_speed_limit, change_speed_limit);
  double max_cruise_speed = common::config::CommonConfig::Instance()
                                ->drive_strategy_config()
                                .motor_way.max_cruise_speed;
  double acc_limit = veh_v_ * veh_v_ / 2.0 / dis_to_change_end;
  auto &max_deceleration =
      plan_config_ptr_->motorway_lane_change_scenario.max_deceleration;
  if (acc_limit > max_deceleration) {
    speed_limit = veh_v_;
    acc_limit = -max_deceleration;
  } else {
    acc_limit = 0.0;
  }
  LOG_INFO(
      "speed_limit: {:.4f}, change_speed_limit: {:.4f}, dis_to_change_end: "
      "{:.4f}, acc_limit: {:.4f}",
      speed_limit, change_speed_limit, dis_to_change_end, acc_limit);

  if (speed_limit < max_cruise_speed) {
    neodrive::global::planning::SpeedLimit internal_speed_limit{};
    internal_speed_limit.set_source_type(SpeedLimitType::LANE_CHANGE);
    internal_speed_limit.add_upper_bounds(speed_limit);
    internal_speed_limit.set_constraint_type(SpeedLimitType::HARD);
    internal_speed_limit.set_acceleration(acc_limit);
    data_center_->mutable_behavior_speed_limits()->SetSpeedLimit(
        internal_speed_limit);
    LOG_INFO("end of lane change dist wait");
  }

  if (CheckFinishTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_WAITING_FINISH;
    return;
  }
  if (CheckCancelTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_WAITING_CANCEL;
    return;
  }
  if (CheckChangingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_WAITING_CHANGING;
    return;
  }
}

void ScenarioMotorwayLaneChangeDecider::OnHandleStateChanging() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  auto &inside_data = data_center_->last_frame()->inside_planner_data();

  auto &lane_change_conf = plan_config_ptr_->motorway_lane_change_scenario;
  double buff =
      inside_data.vel_v < lane_change_conf.buff_of_minspeed
          ? lane_change_conf.buff_of_minspeed
          : lane_change_conf.buff_of_minspeed +
                (lane_change_conf.buff_of_maxspeed -
                 lane_change_conf.buff_of_minspeed) /
                    (lane_change_conf.maxspeed - lane_change_conf.minspeed) *
                    (inside_data.vel_v - lane_change_conf.minspeed);
  LOG_INFO(
      "conf buff_of_minspeed: {:.4f}, buff_of_maxspeed: {:.4f}, "
      "minspeed: {:.4f}, maxspeed: {:.4f}",
      lane_change_conf.buff_of_minspeed, lane_change_conf.buff_of_minspeed,
      lane_change_conf.minspeed, lane_change_conf.maxspeed);
  LOG_INFO("buff: {:.4f}, v: {:.4f}", buff, inside_data.vel_v);
  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      observe_ref_l =
          std::min(task_info.curr_sl().l() + buff, target_point_.l());
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      observe_ref_l =
          std::max(task_info.curr_sl().l() - buff, target_point_.l());
      break;
    default:
      observe_ref_l = 0.0;
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      break;
  }

  if (CheckFinishTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_CHANGING_FINISH;
    return;
  }
  if (CheckCancelTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_CHANGING_CANCEL;
    return;
  }
  if (CheckWaitingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_CHANGING_WAITING;
    return;
  }
}

void ScenarioMotorwayLaneChangeDecider::OnHandleStateFinish() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();

  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  observe_ref_l = target_point_.l();
}

void ScenarioMotorwayLaneChangeDecider::OnHandleStateCancel() {
  LOG_INFO("start to OnHandleStateCancel");
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();

  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  observe_ref_l = 0.0;
  data_center_->mutable_event_report_proxy()->SetEvent(
      EventType::LANE_CHANGE_FAIL);
  LOG_INFO("lane change failed");
  if (CheckCancelFinished()) {
    is_state_change_ = true;
    data_center_->mutable_event_report_proxy()->EventReset(
        EventType::LANE_CHANGE_FAIL);
    LOG_INFO("lane change failed reset");
    curr_decision_ = MotorwayLaneChangeStageChangeFlag::T_CANCEL_PREPARE;
  }
}

bool ScenarioMotorwayLaneChangeDecider::CheckPrepareFinished() { return false; }

bool ScenarioMotorwayLaneChangeDecider::CheckWaitingFinished() { return false; }

bool ScenarioMotorwayLaneChangeDecider::CheckChangingFinished() {
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::CheckFinishFinished() { return false; }

bool ScenarioMotorwayLaneChangeDecider::CheckCancelFinished() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  if (motorway_lane_change_context->is_front_has_lane_turn ||
      motorway_lane_change_context->is_front_has_road_boundary ||
      motorway_lane_change_context->is_front_has_traffic_light)
    return false;
  else {
    LOG_INFO("CancelFinished");
    return true;
  }
}

bool ScenarioMotorwayLaneChangeDecider::CheckWaitingTriggered() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  if (motorway_lane_change_context->is_target_lane_dynamic_obs_clear &&
      motorway_lane_change_context->is_target_lane_static_obs_clear) {
    LOG_INFO("target_lane is clear");
    return false;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  switch (data_center_->navigation_result().navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("target_lane is not clear but adc is on lane.");
        return false;
      }
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("target_lane is not clear but adc is on lane.");
        return false;
      }
      break;
    default:
      LOG_INFO("navigator_request: {}",
               data_center_->navigation_result().navigator_request);
      break;
  }
  return true;
}

bool ScenarioMotorwayLaneChangeDecider::CheckChangingTriggered() {
  if (dis_to_change_end_ < 0.0) {
    LOG_INFO("adc is over end point. dis_to_change_end: {:.4f}",
             dis_to_change_end_);
    return false;
  }
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &lane_change_enable = task_info.current_frame()
                                 ->mutable_outside_planner_data()
                                 ->lane_change_enable_info;

  auto navigator_request = data_center_->navigation_result().navigator_request;
  auto enable_change =
      navigator_request == NavigatorLaneChangeRequest::REQUEST_LEFT_LANE_CHANGE
          ? lane_change_enable.left_lane_change_enable
          : lane_change_enable.right_lane_change_enable;

  switch (navigator_request) {
    case REQUEST_LEFT_LANE_CHANGE:
      if (motorway_lane_change_context->is_target_lane_dynamic_obs_clear &&
          motorway_lane_change_context->is_target_lane_static_obs_clear &&
          enable_change) {
        return true;
      }
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("adc is on lane.");
        return true;
      }
      break;
    case REQUEST_RIGHT_LANE_CHANGE:
      if (motorway_lane_change_context->is_target_lane_dynamic_obs_clear &&
          motorway_lane_change_context->is_target_lane_static_obs_clear &&
          enable_change) {
        return true;
      }
      if (motorway_lane_change_context->is_adc_over_current_lane) {
        LOG_INFO("adc is on lane.");
        return true;
      }
      break;
    default:
      LOG_INFO("navigator_request: {}", navigator_request);
      break;
  }
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::CheckFinishTriggered() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  if (motorway_lane_change_context->is_adc_on_target_lane) return true;
  return false;
}

bool ScenarioMotorwayLaneChangeDecider::CheckCancelTriggered() {
  auto motorway_lane_change_context =
      data_center_->mutable_master_info()
          ->mutable_motorway_lane_change_context();
  if (dis_to_change_end_ < 0.0 &&
      !motorway_lane_change_context->is_adc_over_current_lane) {
    LOG_INFO("adc is over end point. dis_to_change_end: {:.4f}",
             dis_to_change_end_);
    return true;
  }
  return false;
}

void ScenarioMotorwayLaneChangeDecider::SaveMonitorMessage() {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  auto &observe_ref_l = task_info.current_frame()
                            ->mutable_outside_planner_data()
                            ->path_observe_ref_l_info.observe_ref_l;
  const auto &motorway_lane_change_context =
      data_center_->master_info().motorway_lane_change_context();

  static char str_buffer[256 * 2];
  sprintf(str_buffer,
          "[CHANGE_LANE][near obs: %d] [far obs: %d] [target l: %f] "
          "[static_obs_clear: %d] [dynamic_obs_clear: %d] "
          "[adc_on_current_lane: %d] [adc_over_current_lane: %d] "
          "[adc_on_target_lane: %d] [front_has_lane_turn: %d] "
          "[front_has_road_boundary: %d] [front_has_divider_restrict: "
          "%d] [front_has_traffic_light: %d] [dis_to_change_end: %f] "
          "[change_need_dis: %f]",
          near_obstacles_.size(), far_obstacles_.size(), observe_ref_l,
          motorway_lane_change_context.is_target_lane_static_obs_clear,
          motorway_lane_change_context.is_target_lane_dynamic_obs_clear,
          motorway_lane_change_context.is_adc_on_current_lane,
          motorway_lane_change_context.is_adc_over_current_lane,
          motorway_lane_change_context.is_adc_on_target_lane,
          motorway_lane_change_context.is_front_has_lane_turn,
          motorway_lane_change_context.is_front_has_road_boundary,
          motorway_lane_change_context.is_front_has_divider_restrict,
          motorway_lane_change_context.is_front_has_traffic_light,
          dis_to_change_end_, change_need_dis_);
  DataCenter::Instance()->SetMonitorString(
      str_buffer, MonitorItemSource::MOTORWAY_CHANGE_LANE);
}

}  // namespace planning
}  // namespace neodrive
