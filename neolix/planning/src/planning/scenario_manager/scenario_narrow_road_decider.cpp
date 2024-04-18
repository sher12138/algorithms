#include "scenario_narrow_road_decider.h"

namespace neodrive {
namespace planning {

// using neodrive::global::planning::NarrowRoadStageChangeFlag;
// using neodrive::global::planning::NarrowRoadStageState;

ScenarioNarrowRoadDecider::ScenarioNarrowRoadDecider() {
  name_ = "ScenarioNarrowRoadDecider";
}

bool ScenarioNarrowRoadDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/narrow_road_stage";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioNarrowRoadDecider init.");
  return true;
}

bool ScenarioNarrowRoadDecider::ResetStateMachine() {
  // init state, decision.
  // curr_state_ = NarrowRoadStageState::INIT;
  // curr_state_str_ = NarrowRoadStageState::State_Name(curr_state_);
  // prev_decision_ = NarrowRoadStageChangeFlag::T0_NONE_INIT;
  // curr_decision_ = NarrowRoadStageChangeFlag::T0_NONE_INIT;
  // if (!state_machine_.SetInitializeState(curr_state_str_)) {
  //   LOG_WARN("set state machine initialize state failed!");
  //   return false;
  // }
  return true;
}

bool ScenarioNarrowRoadDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioNarrowRoadDecider.");
  return true;
}

ErrorCode ScenarioNarrowRoadDecider::RunOnce() {
  LOG_INFO("ScenarioNarrowRoadDecider::RunOnce");
  // if (OnHandleAllStates() == false) {
  //   auto it = state_machine_function_map_.find(curr_state_);
  //   if (it != state_machine_function_map_.end()) {
  //     auto func = it->second;
  //     (this->*func)();
  //   } else {
  //     // missing handle state function
  //     LOG_ERROR("missing handle state function {}", curr_state_str_);
  //   }
  // }
  // if (!UpdateStateMachine()) {
  //   return ErrorCode::PLANNING_ERROR_FAILED;
  // }
  return ErrorCode::PLANNING_OK;
}

void ScenarioNarrowRoadDecider::RegisterStateMachineResponseFunctions() {
  // state_machine_function_map_.insert(
  //     std::pair<NarrowRoadStageState::State,
  //               void (ScenarioNarrowRoadDecider::*)()>(
  //         NarrowRoadStageState::INIT,
  //         &ScenarioNarrowRoadDecider::OnHandleStateInit));
  // state_machine_function_map_.insert(
  //     std::pair<NarrowRoadStageState::State,
  //               void (ScenarioNarrowRoadDecider::*)()>(
  //         NarrowRoadStageState::DEFAULT,
  //         &ScenarioNarrowRoadDecider::OnHandleStateDefault));
}

bool ScenarioNarrowRoadDecider::UpdateStateMachine() {
  // if (prev_decision_ == curr_decision_) {
  //   return true;
  // }
  // auto prev_state = curr_state_str_;
  // if (!state_machine_.ChangeState(curr_decision_)) {
  //   LOG_ERROR(
  //       "{}: state change fail. state: {}, changeflag: {}", name_,
  //       curr_state_str_,
  //       NarrowRoadStageChangeFlag::ChangeFlag_Name(curr_decision_));
  //   return false;
  // } else {
  //   curr_state_str_ = state_machine_.GetCurrentStateString();
  //   curr_state_ = static_cast<NarrowRoadStageState::State>(
  //       state_machine_.GetCurrentState());
  //   prev_decision_ = curr_decision_;
  //   LOG_INFO("{}: update current_decision: ({}) {} {}", Name(),
  //            state_machine_.GetChangeFlagStr(curr_decision_),
  //            prev_state, curr_state_str_);
  // }
  return true;
}

bool ScenarioNarrowRoadDecider::OnHandleAllStates() { return false; }

void ScenarioNarrowRoadDecider::OnHandleStateInit() {
  // curr_decision_ = NarrowRoadStageChangeFlag::T1_INIT_DEFAULT;
}

void ScenarioNarrowRoadDecider::OnHandleStateDefault() { return; }

}  // namespace planning
}  // namespace neodrive
