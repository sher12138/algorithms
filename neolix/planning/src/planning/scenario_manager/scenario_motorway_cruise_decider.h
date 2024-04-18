#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
enum DrivingDir { NONE = 0, CENTER = 1, LEFT = 2, RIGHT = 3 };
class ScenarioMotorwayCruiseDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioMotorwayCruiseDecider);

  using MotorwayCruiseStageState =
      neodrive::global::planning::MotorwayCruiseStageState;
  using MotorwayCruiseStageChangeFlag =
      neodrive::global::planning::MotorwayCruiseStageChangeFlag;
  using MotorwayLaneChangeStageState =
      neodrive::global::planning::MotorwayLaneChangeStageState;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 public:
  inline const MotorwayCruiseStageState::State& curr_state() {
    return data_center_->mutable_master_info()->motorway_cruise_context().stage;
  }
  inline void set_curr_state(const MotorwayCruiseStageState::State& state) {
    data_center_->mutable_master_info()
        ->mutable_motorway_cruise_context()
        ->stage = state;
  }

 private:
  void RegisterStateMachineResponseFunctions();
  bool ResetStateMachine();
  bool UpdateStateMachine();

  void OnHandleStateKeep();
  void OnHandleStateCentering();
  void OnHandleStateLeftDriving();
  void OnHandleStateRightDriving();

  bool CheckKeepFinished();
  bool CheckCenteringTriggered();
  bool CheckCenteringFinished();
  bool CheckLeftDrivingTriggered();
  bool CheckLeftDrivingFinished();
  bool CheckRightDrivingTriggered();
  bool CheckRightDrivingFinished();

  void SaveMonitorMessage(){};

 private:
  MotorwayCruiseStageChangeFlag::ChangeFlag prev_decision_{
      MotorwayCruiseStageChangeFlag::T_NONE_KEEP};
  MotorwayCruiseStageChangeFlag::ChangeFlag curr_decision_{
      MotorwayCruiseStageChangeFlag::T_NONE_KEEP};
  MotorwayCruiseStageChangeFlag::ChangeFlag filter_decision_{
      MotorwayCruiseStageChangeFlag::T_NONE_KEEP};
  const config::AutoPlanConfig* plan_config_ptr_{nullptr};
  std::unordered_map<MotorwayCruiseStageState::State,
                     void (ScenarioMotorwayCruiseDecider::*)()>
      state_machine_function_map_;
  const neodrive::common::config::AutoDriveStrategy::MotorWay&
      motorway_config_ = common::config::CommonConfig::Instance()
                             ->drive_strategy_config()
                             .motor_way;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioMotorwayCruiseDecider);

}  // namespace planning
}  // namespace neodrive