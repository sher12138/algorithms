#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "planning/scenario_manager/scenario_stage_decider_interface.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {
class ScenarioMotorwayDetourDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioMotorwayDetourDecider);

  using MotorwayDetourStageState =
      neodrive::global::planning::MotorwayDetourStageState;
  using MotorwayDetourStageChangeFlag =
      neodrive::global::planning::MotorwayDetourStageChangeFlag;

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
  void OnHandleStatePrepare();
  void OnHandleStateBorrowing();
  void OnHandleStateReverseLaneBorrowing();
  void OnHandleStateExit();

 private:
  bool ProcessStagePrepare();
  bool ProcessStageBorrowing();
  bool ProcessStageReverseLaneBorrowing();
  bool ProcessStageExit();
  bool ProcessAllStage();

  void SetLaneBorrowTurnType();
  void SetLaneBorrowLaneExtendRatio();
  bool ModifyLaneBound(const TaskInfo& task_info);

  void SaveMonitorMessage();

  bool ExtendBorrowBound(const ReferencePoint& point, const bool left,
                         const bool right, double* left_bound,
                         double* right_bound, bool* left_lane_borrow_flag,
                         bool* right_lane_borrow_flag);

  bool FindMinLaneBound();

  bool IsAdcInLaneWithoutRoadBoundSpeedLimit();

  bool IsReverseLaneDetour(const ReferencePoint& ref_pt);

 public:
  // MotorwayDetourStageState::State curr_state_;
  // curr_state in LaneBorrowContext.
  inline const MotorwayDetourStageState::State& curr_state() {
    return data_center_->mutable_master_info()
        ->motorway_lane_borrow_context()
        .stage;
  }
  inline void set_curr_state(const MotorwayDetourStageState::State& state) {
    data_center_->mutable_master_info()
        ->mutable_motorway_lane_borrow_context()
        ->stage = state;
  }

 private:
  static constexpr int kFilterFrameThreshold = 3;
  static constexpr double kDeltWeightExtend{0.033};
  static constexpr double kDeltWeightShrink{0.033 * 1.5};
  static constexpr double kWeightThresh{1.0 - 0.0001};

 private:
  MotorwayDetourStageChangeFlag::ChangeFlag prev_decision_{
      MotorwayDetourStageChangeFlag::T_NONE_INIT};
  MotorwayDetourStageChangeFlag::ChangeFlag curr_decision_{
      MotorwayDetourStageChangeFlag::T_NONE_INIT};
  MotorwayDetourStageChangeFlag::ChangeFlag filter_decision_{
      MotorwayDetourStageChangeFlag::T_NONE_INIT};  // for filter decision.

  std::unordered_map<MotorwayDetourStageState::State,
                     void (ScenarioMotorwayDetourDecider::*)()>
      state_machine_function_map_;

  const neodrive::common::config::AutoDriveStrategy::MotorWay&
      motorway_config_ = common::config::CommonConfig::Instance()
                             ->drive_strategy_config()
                             .motor_way;

  const config::AutoPlanConfig* plan_config_ptr_{nullptr};
  double start_l_{0.};
  double lane_bound_weight_{1.0};
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioMotorwayDetourDecider);

}  // namespace planning
}  // namespace neodrive