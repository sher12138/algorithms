#pragma once

#include "common/visualizer_event/visualizer_event.h"
#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioBarrierGateDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioBarrierGateDecider);

  using BarrierGateStageState =
      neodrive::global::planning::BarrierGateStageState;
  using BarrierGateStageChangeFlag =
      neodrive::global::planning::BarrierGateStageChangeFlag;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 public:
  inline const BarrierGateStageState::State& curr_state() {
    return data_center_->mutable_master_info()->barrier_gate_context().stage;
  }
  inline void set_curr_state(const BarrierGateStageState::State& state) {
    data_center_->mutable_master_info()->mutable_barrier_gate_context()->stage =
        state;
  }
  inline const bool& is_barrier_gate_finished() {
    return data_center_->mutable_master_info()
        ->barrier_gate_context()
        .is_barrier_gate_finished;
  }
  inline void set_is_barrier_gate_finished(const bool& flag) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->is_barrier_gate_finished = flag;
  }
  inline const bool& consider_barrier_gate() {
    return data_center_->mutable_master_info()
        ->barrier_gate_context()
        .consider_barrier_gate;
  }
  inline void set_consider_barrier_gate(const bool& flag) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->consider_barrier_gate = flag;
  }
  inline const bool& is_move_stage() {
    return data_center_->mutable_master_info()
        ->barrier_gate_context()
        .is_move_stage;
  }
  inline void set_is_move_stage(const bool& flag) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->is_move_stage = flag;
  }
  inline const bool& is_wait_stage() {
    return data_center_->mutable_master_info()
        ->barrier_gate_context()
        .is_wait_stage;
  }
  inline void set_is_wait_stage(const bool& flag) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->is_wait_stage = flag;
  }
  inline const bool& is_stop_stage() {
    return data_center_->mutable_master_info()
        ->barrier_gate_context()
        .is_stop_stage;
  }
  inline void set_is_stop_stage(const bool& flag) {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->is_stop_stage = flag;
  }
  inline void print_context() {
    LOG_INFO(
        "stage: {}, consider_barrier_gate: {}, is_barrier_gate_finished: {}, "
        "curr_decision: {}, is_state_change: {}",
        curr_state_str_, consider_barrier_gate(), is_barrier_gate_finished(),
        state_machine_.GetChangeFlagStr(curr_decision_), is_state_change_);
  }
  inline void reset_context() {
    data_center_->mutable_master_info()
        ->mutable_barrier_gate_context()
        ->Reset();
  }

 private:
  void RegisterStateMachineResponseFunctions();
  bool ResetStateMachine();
  bool UpdateStateMachine();

  void OnHandleStateInit();
  void OnHandleStateStop();
  void OnHandleStateWait();
  void OnHandleStateMove();
  void OnHandleStateExit();

  bool CheckInitFinished();
  bool CheckStopFinished();
  bool CheckWaitFinished();
  bool CheckMoveFinished();
  bool CheckExitFinished();

  bool CheckExitTriggered();

  void VisBarrierGate();

 private:
  static constexpr int kFilterFrameThreshold = 3;
  BarrierGateStageChangeFlag::ChangeFlag prev_decision_{
      BarrierGateStageChangeFlag::T_NONE_INIT};
  BarrierGateStageChangeFlag::ChangeFlag curr_decision_{
      BarrierGateStageChangeFlag::T_NONE_INIT};
  BarrierGateStageChangeFlag::ChangeFlag filter_decision_{
      BarrierGateStageChangeFlag::T_NONE_INIT};
  const config::AutoPlanConfig* plan_config_ptr_{nullptr};
  std::unordered_map<BarrierGateStageState::State,
                     void (ScenarioBarrierGateDecider::*)()>
      state_machine_function_map_;

  int cnt_;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioBarrierGateDecider);

}  // namespace planning
}  // namespace neodrive
