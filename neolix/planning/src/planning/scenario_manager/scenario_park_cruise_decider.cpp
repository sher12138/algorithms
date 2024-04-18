#include "scenario_park_cruise_decider.h"

namespace neodrive {
namespace planning {

ScenarioParkCruiseDecider::ScenarioParkCruiseDecider() { name_ = "ScenarioParkCruiseDecider"; }

bool ScenarioParkCruiseDecider::Init() {
  if (initialized_) {
    return true;
  }
  curr_state_str_ = "DEFAULT";
  initialized_ = true;
  return true;
}

bool ScenarioParkCruiseDecider::Reset() {
  return true;
}

ErrorCode ScenarioParkCruiseDecider::RunOnce() {
  LOG_INFO("ScenarioParkCruiseDecider::RunOnce");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
