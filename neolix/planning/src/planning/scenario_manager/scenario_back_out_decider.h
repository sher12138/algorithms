#pragma once

#include "scenario_manager_msgs.pb.h"
#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
class ScenarioBackOutDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioBackOutDecider);

  using BackOutStageState =
    neodrive::global::planning::BackOutStageState;
  using BackOutStageChangeFlag =
    neodrive::global::planning::BackOutStageChangeFlag;
 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 private:
  bool ResetStateMachine();
  void RegisterStateMachineResponseFunctions();
  bool UpdateStateMachine();

  bool OnHandleAllStates();
  void OnHandleStatePrepare();
  void OnHandleStateBackward();
  void OnHandleStateForward();
  void OnHandleStateExit();

  BackOutStageState::State curr_state_;
  BackOutStageChangeFlag::ChangeFlag prev_decision_;
  BackOutStageChangeFlag::ChangeFlag curr_decision_;

  std::unordered_map<BackOutStageState::State,
                     void (ScenarioBackOutDecider::*)()>
      state_machine_function_map_;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioBackOutDecider);

}  // namespace planning
}  // namespace neodrive
