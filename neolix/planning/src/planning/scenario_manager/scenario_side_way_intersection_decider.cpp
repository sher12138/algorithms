#include "scenario_side_way_intersection_decider.h"

namespace neodrive {
namespace planning {

ScenarioSideWayIntersectionDecider::ScenarioSideWayIntersectionDecider() {
  name_ = "ScenarioSideWayIntersectionDecider";
}

bool ScenarioSideWayIntersectionDecider::Init() {
  if (initialized_) {
    return true;
  }
  // init state machine.
  std::string state_machine_file =
      "/home/caros/cyberrt/conf/state_machine/side_way_intersection";
  if (!state_machine_.LoadStateMachine(state_machine_file)) {
    LOG_WARN("load state machine {} failed!", state_machine_file);
    return false;
  }

  RegisterStateMachineResponseFunctions();
  if (!ResetStateMachine()) {
    return false;
  }

  initialized_ = true;
  LOG_INFO("ScenarioSideWayIntersectionDecider init.");
  return true;
}

bool ScenarioSideWayIntersectionDecider::ResetStateMachine() { return true; }

bool ScenarioSideWayIntersectionDecider::Reset() {
  if (!ResetStateMachine()) {
    return false;
  }
  LOG_INFO("Reset ScenarioSideWayIntersectionDecider.");
  return true;
}

ErrorCode ScenarioSideWayIntersectionDecider::RunOnce() {
  LOG_INFO("ScenarioSideWayIntersectionDecider::RunOnce");

  return ErrorCode::PLANNING_OK;
}

void ScenarioSideWayIntersectionDecider::
    RegisterStateMachineResponseFunctions() {}

bool ScenarioSideWayIntersectionDecider::UpdateStateMachine() {
  // }
  return true;
}

bool ScenarioSideWayIntersectionDecider::OnHandleAllStates() { return false; }

void ScenarioSideWayIntersectionDecider::OnHandleStateInit() {}

void ScenarioSideWayIntersectionDecider::OnHandleStateDefault() { return; }

}  // namespace planning
}  // namespace neodrive