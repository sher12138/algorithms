#pragma once

#include "scenario_manager_msgs.pb.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioNarrowRoadDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioNarrowRoadDecider);

  // using NarrowRoadStageState =
  // neodrive::global::planning::NarrowRoadStageState;
  // using NarrowRoadStageChangeFlag =
  //     neodrive::global::planning::NarrowRoadStageChangeFlag;

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

  // NarrowRoadStageState::State curr_state_;
  // NarrowRoadStageChangeFlag::ChangeFlag prev_decision_;
  // NarrowRoadStageChangeFlag::ChangeFlag curr_decision_;

  // std::unordered_map<NarrowRoadStageState::State,
  //                    void (ScenarioNarrowRoadDecider::*)()>
  //     state_machine_function_map_;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioNarrowRoadDecider);

}  // namespace planning
}  // namespace neodrive
