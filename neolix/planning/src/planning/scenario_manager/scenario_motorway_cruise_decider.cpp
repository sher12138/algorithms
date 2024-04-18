#include "scenario_motorway_cruise_decider.h"

namespace neodrive {
namespace planning {

ScenarioMotorwayCruiseDecider::ScenarioMotorwayCruiseDecider() {
  name_ = "ScenarioMotorwayCruiseDecider";
}

bool ScenarioMotorwayCruiseDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine,
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/scenario_motorway_cruise_stage";
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
  LOG_INFO("ScenarioMotorwayCruiseDecider init.");
  return true;
}

bool ScenarioMotorwayCruiseDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioMotorwayCruiseDecider.");
  return true;
}

ErrorCode ScenarioMotorwayCruiseDecider::RunOnce() {
  LOG_INFO("ScenarioMotorwayDecider::RunOnce");
  LOG_INFO("Motorway Cruise cur stage: {}", curr_state());
  // change state.
  is_state_change_ = false;
  auto it = state_machine_function_map_.find(curr_state());
  if (it != state_machine_function_map_.end()) {
    auto func = it->second;
    (this->*func)();
  } else {
    LOG_ERROR("missing handle state function {}", curr_state_str_);
  }
  if (is_state_change_) {
    if (!UpdateStateMachine()) {
      return ErrorCode::PLANNING_ERROR_FAILED;
    }
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  SaveMonitorMessage();
  return ErrorCode::PLANNING_OK;
}

bool ScenarioMotorwayCruiseDecider::ResetStateMachine() {
  if (!state_machine_.SetInitializeState("KEEP")) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }

  curr_state_str_ = state_machine_.GetCurrentStateString();
  set_curr_state(static_cast<MotorwayCruiseStageState::State>(
      state_machine_.GetCurrentState()));
  prev_decision_ = MotorwayCruiseStageChangeFlag::T_NONE_KEEP;
  curr_decision_ = MotorwayCruiseStageChangeFlag::T_NONE_KEEP;
  return true;
}

void ScenarioMotorwayCruiseDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<MotorwayCruiseStageState::State,
                void (ScenarioMotorwayCruiseDecider::*)()>(
          MotorwayCruiseStageState::KEEP,
          &ScenarioMotorwayCruiseDecider::OnHandleStateKeep));
  state_machine_function_map_.insert(
      std::pair<MotorwayCruiseStageState::State,
                void (ScenarioMotorwayCruiseDecider::*)()>(
          MotorwayCruiseStageState::CENTERING,
          &ScenarioMotorwayCruiseDecider::OnHandleStateCentering));
  state_machine_function_map_.insert(
      std::pair<MotorwayCruiseStageState::State,
                void (ScenarioMotorwayCruiseDecider::*)()>(
          MotorwayCruiseStageState::LEFT_DRIVING,
          &ScenarioMotorwayCruiseDecider::OnHandleStateLeftDriving));
  state_machine_function_map_.insert(
      std::pair<MotorwayCruiseStageState::State,
                void (ScenarioMotorwayCruiseDecider::*)()>(
          MotorwayCruiseStageState::RIGHT_DRIVING,
          &ScenarioMotorwayCruiseDecider::OnHandleStateRightDriving));
}

bool ScenarioMotorwayCruiseDecider::UpdateStateMachine() {
  // filter decision.
  static int decision_filter_count = 0;
  if (prev_decision_ == curr_decision_) {
    // reset count.
    decision_filter_count = 0;
    return true;
  }
  if (curr_decision_ == filter_decision_) {
    ++decision_filter_count;
  } else if (curr_state() != MotorwayCruiseStageState::KEEP) {
    filter_decision_ = curr_decision_;
    decision_filter_count =
        config::PlanningConfig::Instance()
            ->plan_config()
            .motorway_cruise_scenario.filter_frame_threshold;
  } else {
    // new decision, update filter_decision and count 1.
    filter_decision_ = curr_decision_;
    decision_filter_count = 1;
    // ==> debug.
    LOG_INFO("filter decision: {}",
             MotorwayCruiseStageChangeFlag::ChangeFlag_Name(curr_decision_));
  }
  if (decision_filter_count <
      config::PlanningConfig::Instance()
          ->plan_config()
          .motorway_cruise_scenario.filter_frame_threshold) {
    // hold state.
    LOG_INFO("hold state: {}", curr_state());
    return true;
  }
  // update state.
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR("{}: state change fail. state: {}, changeflag: {}", name_,
              curr_state_str_,
              MotorwayCruiseStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    set_curr_state(static_cast<MotorwayCruiseStageState::State>(
        state_machine_.GetCurrentState()));
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

void ScenarioMotorwayCruiseDecider::OnHandleStateKeep() {
  curr_decision_ = MotorwayCruiseStageChangeFlag::T_NONE_KEEP;
  if (!CheckKeepFinished()) {
    is_state_change_ = false;
    return;
  }
  if (CheckCenteringTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_KEEP_CENTERING;
    return;
  }
  if (CheckLeftDrivingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_KEEP_LEFT_DRIVING;
    return;
  }
  if (CheckRightDrivingTriggered()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_KEEP_RIGHT_DRIVING;
    return;
  }
}

void ScenarioMotorwayCruiseDecider::OnHandleStateCentering() {
  if (CheckCenteringFinished()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_CENTERING_KEEP;
    return;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, motorway_config_.enable_bias_drive);
}

void ScenarioMotorwayCruiseDecider::OnHandleStateLeftDriving() {
  if (CheckLeftDrivingFinished()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_LEFT_DRIVING_KEEP;
    return;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, motorway_config_.enable_bias_drive);
}

void ScenarioMotorwayCruiseDecider::OnHandleStateRightDriving() {
  if (CheckRightDrivingFinished()) {
    is_state_change_ = true;
    curr_decision_ = MotorwayCruiseStageChangeFlag::T_RIGHT_DRIVING_KEEP;
    return;
  }
  auto &task_info = data_center_->mutable_task_info_list()->front();
  scenario_common::ComputeRefL(task_info, motorway_config_.enable_bias_drive);
}

bool ScenarioMotorwayCruiseDecider::CheckKeepFinished() {
  if (config::PlanningConfig::Instance()
          ->plan_config()
          .motorway_cruise_scenario.driving_direction != DrivingDir::NONE) {
    return true;
  }
  return false;
}

bool ScenarioMotorwayCruiseDecider::CheckCenteringTriggered() {
  if (!motorway_config_.enable_bias_drive) {
    return true;
  }
  return false;
}

bool ScenarioMotorwayCruiseDecider::CheckCenteringFinished() {
  if (motorway_config_.enable_bias_drive) {
    return true;
  }
  return false;
}

bool ScenarioMotorwayCruiseDecider::CheckLeftDrivingTriggered() {
  return false;
}

bool ScenarioMotorwayCruiseDecider::CheckLeftDrivingFinished() { return true; }

bool ScenarioMotorwayCruiseDecider::CheckRightDrivingTriggered() {
  if (motorway_config_.enable_bias_drive) {
    return true;
  }
  return false;
}

bool ScenarioMotorwayCruiseDecider::CheckRightDrivingFinished() {
  if (!motorway_config_.enable_bias_drive) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive