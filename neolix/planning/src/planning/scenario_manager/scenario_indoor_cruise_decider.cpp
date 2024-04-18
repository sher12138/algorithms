#include "scenario_indoor_cruise_decider.h"

namespace neodrive {
namespace planning {

ScenarioIndoorCruiseDecider::ScenarioIndoorCruiseDecider() { name_ = "ScenarioIndoorCruiseDecider"; }

bool ScenarioIndoorCruiseDecider::Init() {
  if (initialized_) {
    return true;
  }
  curr_state_str_ = "DEFAULT";
  initialized_ = true;
  return true;
}

bool ScenarioIndoorCruiseDecider::Reset() {
  return true;
}

ErrorCode ScenarioIndoorCruiseDecider::RunOnce() {
  LOG_INFO("ScenarioIndoorCruiseDecider::RunOnce");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
