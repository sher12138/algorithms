#include "scenario_back_out_decider.h"

namespace neodrive {
namespace planning {

using neodrive::global::planning::BackOutStageChangeFlag;
using neodrive::global::planning::BackOutStageState;

ScenarioBackOutDecider::ScenarioBackOutDecider() {
  name_ = "ScenarioBackOutDecider";
}

bool ScenarioBackOutDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/back_out";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioBackOutDecider init.");
  return true;
}

bool ScenarioBackOutDecider::ResetStateMachine() {
  // init state, decision.
 	curr_state_ = BackOutStageState::PREPARE;
  curr_state_str_ = BackOutStageState::State_Name(curr_state_);
  prev_decision_ = BackOutStageChangeFlag::T0_PREPARE;
  curr_decision_ = BackOutStageChangeFlag::T0_PREPARE;
  
  if (!state_machine_.SetInitializeState(curr_state_str_)) {
    LOG_WARN("set state machine initialize state failed!");
    return false;
  }
  return true;
}

bool ScenarioBackOutDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioBackOutDecider.");
  return true;
}

ErrorCode ScenarioBackOutDecider::RunOnce() {
  LOG_INFO("ScenarioBackOutDecider::RunOnce");
  if (OnHandleAllStates() == false) {
    auto it = state_machine_function_map_.find(curr_state_);
    if (it != state_machine_function_map_.end()) {
      auto func = it->second;
      (this->*func)();
    } else {
      // missing handle state function
      LOG_ERROR("missing handle state function {}", curr_state_str_);
    }
  }
  if (!UpdateStateMachine()) {
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

void ScenarioBackOutDecider::RegisterStateMachineResponseFunctions() {
  state_machine_function_map_.insert(
      std::pair<BackOutStageState::State,
                void (ScenarioBackOutDecider::*)()>(
          BackOutStageState::PREPARE,
          &ScenarioBackOutDecider::OnHandleStatePrepare));
  state_machine_function_map_.insert(
      std::pair<BackOutStageState::State,
                void (ScenarioBackOutDecider::*)()>(
          BackOutStageState::BACKWARD,
          &ScenarioBackOutDecider::OnHandleStateBackward));
  state_machine_function_map_.insert(
      std::pair<BackOutStageState::State,
                void (ScenarioBackOutDecider::*)()>(
          BackOutStageState::FORWARD,
          &ScenarioBackOutDecider::OnHandleStateForward));
  state_machine_function_map_.insert(
      std::pair<BackOutStageState::State,
                void (ScenarioBackOutDecider::*)()>(
          BackOutStageState::EXIT,
          &ScenarioBackOutDecider::OnHandleStateExit));
}

bool ScenarioBackOutDecider::UpdateStateMachine() {
  if (prev_decision_ == curr_decision_) {
    return true;
  }
  auto prev_state = curr_state_str_;
  if (!state_machine_.ChangeState(curr_decision_)) {
    LOG_ERROR(
        "{}: state change fail. state: {}, changeflag: {}", name_,
        curr_state_str_,
        BackOutStageChangeFlag::ChangeFlag_Name(curr_decision_));
    return false;
  } else {
    curr_state_str_ = state_machine_.GetCurrentStateString();
    curr_state_ = static_cast<BackOutStageState::State>(
        state_machine_.GetCurrentState());
    prev_decision_ = curr_decision_;
    LOG_INFO("{}: update current_decision: ({}) {} {}", Name(),
             state_machine_.GetChangeFlagStr(curr_decision_),
             prev_state, curr_state_str_);
  }
  return true;
}

bool ScenarioBackOutDecider::OnHandleAllStates() { return false; }

void ScenarioBackOutDecider::OnHandleStatePrepare() { return; }

void ScenarioBackOutDecider::OnHandleStateBackward() { return; }

void ScenarioBackOutDecider::OnHandleStateForward() { return; }

void ScenarioBackOutDecider::OnHandleStateExit() { return; }

}  // namespace planning
}  // namespace neodrive
