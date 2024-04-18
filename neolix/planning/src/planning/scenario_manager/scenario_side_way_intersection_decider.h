#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioSideWayIntersectionDecider
    : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioSideWayIntersectionDecider);

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 private:
  bool ResetStateMachine();
  void RegisterStateMachineResponseFunctions();
  bool UpdateStateMachine();

  bool OnHandleAllStates();
  void OnHandleStateInit();
  void OnHandleStateDefault();
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioSideWayIntersectionDecider);

}  // namespace planning
}  // namespace neodrive