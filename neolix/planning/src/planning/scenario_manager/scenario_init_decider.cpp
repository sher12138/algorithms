#include "scenario_init_decider.h"

namespace neodrive {
namespace planning {

ScenarioInitDecider::ScenarioInitDecider() { name_ = "ScenarioInitDecider"; }

bool ScenarioInitDecider::Init() {
  if (initialized_) {
    return true;
  }
  curr_state_str_ = "DEFAULT";
  // TODO:
  initialized_ = true;
  return true;
}

bool ScenarioInitDecider::Reset() {
  //
  return true;
}

ErrorCode ScenarioInitDecider::RunOnce() {
  LOG_INFO("ScenarioInitDecider::RunOnce");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
