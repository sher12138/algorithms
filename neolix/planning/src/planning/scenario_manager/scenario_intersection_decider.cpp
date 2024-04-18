#include "scenario_intersection_decider.h"

namespace neodrive {
namespace planning {

ScenarioIntersectionDecider::ScenarioIntersectionDecider() {
  name_ = "ScenarioIntersectionDecider";
}

bool ScenarioIntersectionDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/intersection_state";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }
  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }
  initialized_ = true;
  LOG_INFO("ScenarioIntersectionDecider init.");
  return true;
}

bool ScenarioIntersectionDecider::ResetStateMachine() {
  // considering the state transform filter, reset state to PREPARE instead of
  // INIT.
  curr_state_ = IntersectionState::INIT;
  if (!state_machine_.SetInitializeState("INIT")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }
  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(
      static_cast<IntersectionState::State>(state_machine_.GetCurrentState()));
  return true;
}

bool ScenarioIntersectionDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioIntersectionDecider.");
  return true;
}

void ScenarioIntersectionDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::INIT,
          &ScenarioIntersectionDecider::OnHandleStateInit));
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::STRAIGHT,
          &ScenarioIntersectionDecider::OnHandleStateStraight));
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::ABOUT_TO_TURN_RIGHT,
          &ScenarioIntersectionDecider::OnHandleStateAboutToTurnRight));
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::TURN_RIGHT,
          &ScenarioIntersectionDecider::OnHandleStateTurnRight));
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::ABOUT_TO_TURN_LEFT,
          &ScenarioIntersectionDecider::OnHandleStateAboutToTurnLeft));
  state_machine_function_map_.insert(
      std::pair<IntersectionState::State,
                void (ScenarioIntersectionDecider::*)()>(
          IntersectionState::TURN_LEFT,
          &ScenarioIntersectionDecider::OnHandleStateTurnLeft));
}

ErrorCode ScenarioIntersectionDecider::RunOnce() {
  LOG_INFO("ScenarioIntersectionDecider::RunOnce");
  is_state_change_ = false;
  data_center_ = DataCenter::Instance();
  current_s_ =
      data_center_->last_frame()->inside_planner_data().init_sl_point.s();
  LOG_INFO("current_s = {:.4f}", current_s_);
  GetRefLineType();
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
  return ErrorCode::PLANNING_OK;
}

bool ScenarioIntersectionDecider::UpdateStateMachine() {
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
    LOG_INFO("filter decision: {}",
             IntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_));
  }
  if (decision_filter_count < kFilterFrameThreshold) {
    // hold state.
    return true;
  }
  LOG_INFO("curr_change_flag = {]}", curr_change_flag_);
  if (!state_machine_.ChangeState(curr_change_flag_)) {
    LOG_ERROR("ChangeState error: {} {}", curr_state_str_,
              IntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_));
    return false;
  }
  auto tmp_state = curr_state_;
  auto tmp_state_str = curr_state_str_;
  curr_state_str_ = state_machine_.GetCurrentStateString();
  curr_state_ =
      static_cast<IntersectionState::State>(state_machine_.GetCurrentState());
  set_curr_state(curr_state_);
  if (tmp_state != curr_state_) {
    prev_state_ = tmp_state;
    prev_state_str_ = tmp_state_str;
    LOG_INFO("state transform: {}, {} -> {} {}",
             IntersectionChangeFlag::ChangeFlag_Name(curr_change_flag_),
             IntersectionState::State_Name(tmp_state),
             IntersectionState::State_Name(curr_state_), curr_state_str_);
  }
  return true;
}

void ScenarioIntersectionDecider::OnHandleStateInit() {
  is_state_change_ = true;
  if (DetectStraight()) {
    curr_change_flag_ = IntersectionChangeFlag::T7_INIT_STRAIGHT;
    return;
  }
  if (DetectTurnRight()) {
    curr_change_flag_ = IntersectionChangeFlag::T8_INIT_TURN_RIGHT;
    return;
  }
  if (DetectTurnLeft()) {
    curr_change_flag_ = IntersectionChangeFlag::T9_INIT_TURN_LEFT;
    return;
  }
}

void ScenarioIntersectionDecider::OnHandleStateStraight() {
  if (DetectAboutToTurnRight()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T0_STRAIGHT_ABOUT_TO_TURN_RIGHT;
    return;
  }
  if (DetectAboutToTurnLeft()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T1_STRAIGHT_ABOUT_TO_TURN_LEFT;
    return;
  }
}

void ScenarioIntersectionDecider::OnHandleStateAboutToTurnRight() {
  if (DetectTurnRight()) {
    is_state_change_ = true;
    curr_change_flag_ =
        IntersectionChangeFlag::T2_ABOUT_TO_TURN_RIGHT_TURN_RIGHT;
    return;
  }
}

void ScenarioIntersectionDecider::OnHandleStateTurnRight() {
  if (DetectTurnRight()) {
    return;
  }
  if (DetectStraight()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T4_TURN_RIGHT_STRAIGHT;
    return;
  }
  if (DetectAboutToTurnLeft()) {
    is_state_change_ = true;
    curr_change_flag_ =
        IntersectionChangeFlag::T6_TURN_RIGHT_ABOUT_TO_TURN_LEFT;
    return;
  }
  if (DetectTurnLeft()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T11_TURN_RIGHT_TURN_LEFT;
    return;
  }
}

void ScenarioIntersectionDecider::OnHandleStateAboutToTurnLeft() {
  if (DetectTurnLeft()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T3_ABOUT_TO_TURN_LEFT_TURN_LEFT;
    return;
  }
}

void ScenarioIntersectionDecider::OnHandleStateTurnLeft() {
  if (DetectTurnLeft()) {
    return;
  }
  if (DetectStraight()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T5_TURN_LEFT_STRAIGHT;
    return;
  }
  if (DetectTurnRight()) {
    is_state_change_ = true;
    curr_change_flag_ = IntersectionChangeFlag::T10_TURN_LEFT_TURN_RIGHT;
    return;
  }
}

bool ScenarioIntersectionDecider::DetectStraight() {
  if (current_type_ == RefLineType::STRAIGHT) {
    return true;
  } else
    return false;
}

bool ScenarioIntersectionDecider::DetectAboutToTurnRight() {
  if (forward_type_ == RefLineType::RIGHT)
    return true;
  else
    return false;
}

bool ScenarioIntersectionDecider::DetectTurnRight() {
  if (current_type_ == RefLineType::RIGHT)
    return true;
  else
    return false;
}

bool ScenarioIntersectionDecider::DetectAboutToTurnLeft() {
  if (forward_type_ == RefLineType::LEFT)
    return true;
  else
    return false;
}

bool ScenarioIntersectionDecider::DetectTurnLeft() {
  if (current_type_ == RefLineType::LEFT)
    return true;
  else
    return false;
}

void ScenarioIntersectionDecider::GetRefLineType() {
  current_type_ =
      CalRefLineType(current_s_, intersection_scenario_config_.detect_length);
  auto &task_info = data_center_->mutable_task_info_list()->front();
  double detect_distance = intersection_scenario_config_.detect_length;
  if (task_info.last_frame() != nullptr) {
    detect_distance =
        std::max(detect_distance,
                 intersection_scenario_config_.approach_time_threshold *
                     task_info.last_frame()->inside_planner_data().vel_v);
  }
  forward_type_ =
      CalRefLineType(current_s_ + intersection_scenario_config_.detect_length,
                     detect_distance);
}

double ScenarioIntersectionDecider::GetKappa(double s) {
  auto &task_info = data_center_->mutable_task_info_list()->front();
  ReferenceLinePtr reference_line = task_info.reference_line();
  ReferencePoint tmp_refer_pt;
  if (!reference_line->GetNearestRefPoint(s, &tmp_refer_pt)) {
    LOG_ERROR("GetNearestRefPoint failed, s: {:.4f}", s);
    return 0.0;
  }
  return tmp_refer_pt.kappa();
}

RefLineType ScenarioIntersectionDecider::CalRefLineType(double start_s,
                                                        double length) {
  double kappa_sum = 0;
  double kappa_ave;
  std::queue<double> kappa_queue;
  for (double forward_s = start_s; forward_s < start_s + length;
       forward_s += 0.2) {
    if (kappa_queue.size() > 10) {
      kappa_sum -= kappa_queue.front();
      kappa_queue.pop();
    }
    kappa_queue.push(GetKappa(forward_s));
    kappa_sum += kappa_queue.back();
    kappa_ave = kappa_sum / kappa_queue.size();
    // turn right, kappa < 0; turn left, kappa > 0
    if (kappa_ave > intersection_scenario_config_.kappa_threshold) {
      return RefLineType::LEFT;
    }
    if (kappa_ave < -intersection_scenario_config_.kappa_threshold) {
      return RefLineType::RIGHT;
    }
  }
  return RefLineType::STRAIGHT;
}
}  // namespace planning

}  // namespace neodrive
