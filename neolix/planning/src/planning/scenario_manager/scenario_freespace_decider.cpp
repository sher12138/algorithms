#include "scenario_freespace_decider.h"

namespace neodrive {
namespace planning {

ScenarioFreespaceDecider::ScenarioFreespaceDecider() {
  name_ = "ScenarioFreespaceDecider";
}

bool ScenarioFreespaceDecider::Init() {
  curr_state_str_ = "DEFAULT";
  initialized_ = true;
  return true;
}

bool ScenarioFreespaceDecider::Reset() { return true; }

ErrorCode ScenarioFreespaceDecider::RunOnce() {
  LOG_INFO("ScenarioFreespaceDecider::RunOnce");
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
