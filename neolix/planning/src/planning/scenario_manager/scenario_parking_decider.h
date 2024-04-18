#pragma once

#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioParkingDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioParkingDecider);

  using ParkingStageState = neodrive::global::planning::ParkingStageState;
  using ParkingStageChangeFlag =
      neodrive::global::planning::ParkingStageChangeFlag;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 private:
 void RegisterStateMachineResponseFunctions();
 bool ResetStateMachine();
 void OnHandleStateInit();
 void OnHandleStateParkingIn();
 void OnHandleStateParkingOut();
 void OnHandleStateFinish();
 bool UpdateStateMachine();
 bool OnHandleAllStates();

 private:
 std::unordered_map<ParkingStageState::State, void (ScenarioParkingDecider::*)()>
      state_machine_function_map_;
 ParkingStageState::State curr_state_;
 ParkingStageChangeFlag::ChangeFlag prev_decision_;
 ParkingStageChangeFlag::ChangeFlag curr_decision_;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioParkingDecider);

}  // namespace planning
}  // namespace neodrive