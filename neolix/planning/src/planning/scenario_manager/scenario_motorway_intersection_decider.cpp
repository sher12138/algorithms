#include "scenario_motorway_intersection_decider.h"

#include "src/planning/scenario_manager/scenario_common.h"

namespace neodrive {
namespace planning {

ScenarioMotorwayIntersectionDecider::ScenarioMotorwayIntersectionDecider() {
  name_ = "ScenarioMotorwayIntersectionDecider";
}

bool ScenarioMotorwayIntersectionDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/"
      "scenario_motorway_intersection_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }
  initialized_ = true;
  LOG_INFO("ScenarioMotorwayIntersectionDecider init.");
  return true;
}

bool ScenarioMotorwayIntersectionDecider::ResetStateMachine() {
  // considering the state transform filter, reset state to PREPARE instead of
  // INIT.
  curr_state_ = MotorwayIntersectionStageState::INIT;
  if (!state_machine_.SetInitializeState("INIT")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }
  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(static_cast<MotorwayIntersectionStageState::State>(
      state_machine_.GetCurrentState()));
  return true;
}

bool ScenarioMotorwayIntersectionDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioMotorwayIntersectionDecider.");
  return true;
}

void ScenarioMotorwayIntersectionDecider::
    RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::INIT,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::STRAIGHT,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateStraight));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::TURN_RIGHT,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateTurnRight));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::LEFT_WAIT_ZONE,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateLeftWaitZone));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::UNPROTECTED_TURN_LEFT,
          &ScenarioMotorwayIntersectionDecider::
              OnHandleStateUnprotectedTurnLeft));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::PROTECTED_TURN_LEFT,
          &ScenarioMotorwayIntersectionDecider::
              OnHandleStateProtectedTurnLeft));
  state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::U_TURN,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateUTurn));
  /*state_machine_function_map_.insert(
      std::pair<MotorwayIntersectionStageState::State,
                void (ScenarioMotorwayIntersectionDecider::*)()>(
          MotorwayIntersectionStageState::EXIT,
          &ScenarioMotorwayIntersectionDecider::OnHandleStateExit));*/
}

ErrorCode ScenarioMotorwayIntersectionDecider::RunOnce() {
  LOG_INFO("ScenarioMotorwayIntersectionDecider::RunOnce");
  is_state_change_ = false;
  data_center_ = DataCenter::Instance();
  auto it = state_machine_function_map_.find(curr_state_);
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
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, motorway_config_.enable_bias_drive);
  return ErrorCode::PLANNING_OK;
}

bool ScenarioMotorwayIntersectionDecider::UpdateStateMachine() {
  static int decision_filter_count = 0;
  if (!is_state_change_) {
    // reset filter.
    decision_filter_count = 0;
    return true;
  }
  if (curr_change_flag_ == filter_change_flag_) {
    ++decision_filter_count;
  } else {
    // new decision, update filter_change_flag and count 1.
    filter_change_flag_ = curr_change_flag_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO(
        "filter decision: {}",
        MotorwayIntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state.
    return true;
  }
  LOG_INFO("curr_change_flag = {]}", curr_change_flag_);
  if (!state_machine_.ChangeState(curr_change_flag_)) {
    LOG_ERROR(
        "ChangeState error: {} {}", curr_state_str_,
        MotorwayIntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_));
    return false;
  }
  auto tmp_state = curr_state_;
  auto tmp_state_str = curr_state_str_;
  curr_state_str_ = state_machine_.GetCurrentStateString();
  curr_state_ = static_cast<MotorwayIntersectionStageState::State>(
      state_machine_.GetCurrentState());
  set_curr_state(curr_state_);
  if (tmp_state != curr_state_) {
    prev_state_ = tmp_state;
    prev_state_str_ = tmp_state_str;
    LOG_INFO("state transform: {}, {} -> {} {}",
             MotorwayIntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_),
             MotorwayIntersectionStageState::State_Name(tmp_state),
             MotorwayIntersectionStageState::State_Name(curr_state_),
             curr_state_str_);
  }
  return true;
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateInit() {
  is_state_change_ = true;
  auto &task_info = data_center_->task_info_list().front();
  double detect_distance =
      config::PlanningConfig::Instance()
          ->plan_config()
          .intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance =
        std::max(detect_distance,
                 config::PlanningConfig::Instance()
                         ->plan_config()
                         .intersection_scenario.approach_time_threshold *
                     task_info.last_frame()->inside_planner_data().vel_v);
  }
  ref_line_type_ = GetRefLineType(
      motor_intersection_scenario_.lanetype_detect_distance + detect_distance);
  have_signal_ = HaveSignal();
  if (DetectStraight()) {
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_INIT_STRAIGHT;
    return;
  }
  if (DetectTurnRight()) {
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_INIT_TURN_RIGHT;
    return;
  }
  if (DetectLeftWaitZone()) {
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_INIT_LEFT_WAIT_ZONE;
    return;
  }
  if (DetectUnprotectedTurnLeft()) {
    curr_change_flag_ =
        MotorwayIntersectionChangeFlag::T_INIT_UNPROTECTED_TURN_LEFT;
    return;
  }
  if (DetectProtectedTurnLeft()) {
    curr_change_flag_ =
        MotorwayIntersectionChangeFlag::T_INIT_PROTECTED_TURN_LEFT;
    return;
  }
  if (DetectUTurn()) {
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_INIT_U_TURN;
    return;
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateStraight() {
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_STRAIGHT_EXIT;
    return;
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateTurnRight() {
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_TURN_RIGHT_EXIT;
    return;
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateLeftWaitZone() {
  ref_line_type_ = GetRefLineType();
  have_signal_ = HaveSignal();
  if (DetectProtectedTurnLeft()) {
    is_state_change_ = true;
    curr_change_flag_ =
        MotorwayIntersectionChangeFlag::T_LEFT_WAIT_ZONE_PROTECTED_TURN_LEFT;
    return;
  }
  if (DetectUTurn()) {
    is_state_change_ = true;
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_LEFT_WAIT_ZONE_U_TURN;
    return;
  }
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_LEFT_WAIT_ZONE_EXIT;
    return;
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateUTurn() {
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ = MotorwayIntersectionChangeFlag::T_U_TURN_EXIT;
    return;
  }

  double min_width = 2 * VehicleParam::Instance()->min_turn_radius() +
                     VehicleParam::Instance()->width() + 1.0;
  LOG_INFO(
      "min_turn_radius: {:.4f}, uturn_width: {:.4f},uturn need min_width: "
      "{:.4f}",
      VehicleParam::Instance()->min_turn_radius(), uturn_width_, min_width);
  if (uturn_width_ < min_width) {
    double buff = std::min(uturn_width_ - min_width, 0.0) / 2.0;
    auto &task_info = data_center_->mutable_task_info_list()->front();
    std::size_t ref_points_size =
        task_info.reference_line()->ref_points().size();
    for (std::size_t i = 0; i < ref_points_size; ++i) {
      auto &pt_utm = task_info.reference_line_raw()->ref_points().at(i);
      task_info.reference_line()->SetIndexBound(
          i, pt_utm.left_lane_bound(),
          std::min(pt_utm.right_lane_bound() - buff,
                   pt_utm.right_road_bound()));
      task_info.reference_line_raw()->SetIndexBound(
          i, pt_utm.left_lane_bound(),
          std::min(pt_utm.right_lane_bound() - buff,
                   pt_utm.right_road_bound()));
      task_info.reference_line()->SetIndexLaneBorrowFlag(i, true, true);
      task_info.reference_line_raw()->SetIndexLaneBorrowFlag(i, true, true);
    }
    task_info.current_frame()
        ->mutable_outside_planner_data()
        ->path_observe_ref_l_info.observe_ref_l =
        std::min(task_info.curr_sl().l(),
                 task_info.curr_referline_pt().left_bound() - min_width / 2.0);
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateUnprotectedTurnLeft() {
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ =
        MotorwayIntersectionChangeFlag::T_UNPROTECTED_TURN_LEFT_EXIT;
    return;
  }
}

void ScenarioMotorwayIntersectionDecider::OnHandleStateProtectedTurnLeft() {
  if (DetectExit()) {
    is_state_change_ = true;
    curr_change_flag_ =
        MotorwayIntersectionChangeFlag::T_PROTECTED_TURN_LEFT_EXIT;
    return;
  }
}

bool ScenarioMotorwayIntersectionDecider::DetectStraight() {
  if (ref_line_type_ == RefLineType::STRAIGHT)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectTurnRight() {
  if (ref_line_type_ == RefLineType::RIGHT)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectLeftWaitZone() {
  if (ref_line_type_ == RefLineType::TurnWatingZone)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectUTurn() {
  if (ref_line_type_ == RefLineType::UTURN)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectUnprotectedTurnLeft() {
  // no traffic light
  if (ref_line_type_ == RefLineType::LEFT && !have_signal_)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectProtectedTurnLeft() {
  // have traffic light
  if (ref_line_type_ == RefLineType::LEFT && have_signal_)
    return true;
  else
    return false;
}

bool ScenarioMotorwayIntersectionDecider::DetectExit() { return false; }

RefLineType ScenarioMotorwayIntersectionDecider::GetRefLineType(double length) {
  auto &task_info = data_center_->task_info_list().front();
  ReferenceLinePtr reference_line = task_info.reference_line();
  ReferencePoint pt{};
  for (double forward_s = 0.0; forward_s <= length; forward_s += 1.0) {
    if (!reference_line->GetNearestRefPoint(task_info.curr_sl().s() + forward_s,
                                            &pt)) {
      LOG_WARN("GetNearestRefPoint fail");
      continue;
    }
    if (pt.is_no_signal()) {
      continue;
    } else if (pt.lane_type_is_left_turn_wating_zone()) {
      return RefLineType::TurnWatingZone;
    } else if (pt.is_left_signal()) {
      return RefLineType::LEFT;
    } else if (pt.is_right_signal()) {
      return RefLineType::RIGHT;
    } else if (pt.is_uturn_signal()) {
      ReferencePoint pt1{pt};
      ReferencePoint pt2{pt};
      for (; forward_s <= length + 100.0; forward_s += 0.1) {
        if (!reference_line->GetNearestRefPoint(
                task_info.curr_sl().s() + forward_s, &pt)) {
          LOG_WARN("GetNearestRefPoint fail");
          continue;
        }
        if (!pt.is_uturn_signal()) {
          pt2 = pt;
          break;
        }
      }
      uturn_width_ = std::sqrt((pt1.x() - pt2.x()) * (pt1.x() - pt2.x()) +
                               (pt1.y() - pt2.y()) * (pt1.y() - pt2.y())) +
                     pt2.right_lane_bound() + pt1.right_lane_bound();
      LOG_INFO("uturn width: {:.4f}", uturn_width_);
      return RefLineType::UTURN;
    }
  }
  return RefLineType::STRAIGHT;
}

RefLineType ScenarioMotorwayIntersectionDecider::GetRefLineType() {
  auto &task_info = data_center_->task_info_list().front();
  ReferenceLinePtr reference_line = task_info.reference_line();
  ReferencePoint pt{};
  if (!reference_line->GetNearestRefPoint(task_info.curr_sl().s(), &pt)) {
    LOG_WARN("GetNearestRefPoint fail");
    return RefLineType::STRAIGHT;
  }
  if (pt.is_no_signal()) {
    return RefLineType::STRAIGHT;
  } else if (pt.lane_type_is_left_turn_wating_zone()) {
    return RefLineType::TurnWatingZone;
  } else if (pt.is_left_signal()) {
    return RefLineType::LEFT;
  } else if (pt.is_right_signal()) {
    return RefLineType::RIGHT;
  } else if (pt.is_uturn_signal()) {
    return RefLineType::UTURN;
  }
}

bool ScenarioMotorwayIntersectionDecider::HaveSignal() {
  auto &task_info = data_center_->task_info_list().front();
  const double curr_s = task_info.curr_sl().s();
  double detect_distance =
      config::PlanningConfig::Instance()
          ->plan_config()
          .intersection_scenario.approach_distance_threshold;
  if (task_info.last_frame() != nullptr) {
    detect_distance =
        std::max(detect_distance,
                 config::PlanningConfig::Instance()
                         ->plan_config()
                         .intersection_scenario.approach_time_threshold *
                     task_info.last_frame()->inside_planner_data().vel_v);
  }
  for (auto [id, s, e] : task_info.reference_line()->signal_overlaps()) {
    if (e < curr_s - motor_intersection_scenario_.signal_detect_distance)
      continue;
    if (s > curr_s + motor_intersection_scenario_.signal_detect_distance +
                detect_distance)
      return false;
    LOG_INFO("find front traffic_light {}, {}, {}", id, s, e);
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive