#pragma once

#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
enum RefLineType {
  STRAIGHT = 0,
  RIGHT = 1,
  LEFT = 2,
  UTURN = 3,
  TurnWatingZone = 4
};
class ScenarioMotorwayIntersectionDecider
    : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioMotorwayIntersectionDecider);

  using MotorwayIntersectionChangeFlag =
      neodrive::global::planning::MotorwayIntersectionStageChangeFlag;
  using MotorwayIntersectionStageState =
      neodrive::global::planning::MotorwayIntersectionStageState;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

  inline const MotorwayIntersectionStageState::State& curr_state() {
    return data_center_->mutable_master_info()
        ->motorway_intersection_context()
        .stage;
  }
  inline void set_curr_state(
      const MotorwayIntersectionStageState::State& state) {
    data_center_->mutable_master_info()
        ->mutable_motorway_intersection_context()
        ->stage = state;
  }

 private:
  bool ResetStateMachine();
  void RegisterStateMachineResponseFunctions();
  bool UpdateStateMachine();
  void OnHandleStateInit();
  void OnHandleStateStraight();
  void OnHandleStateTurnRight();
  void OnHandleStateLeftWaitZone();
  void OnHandleStateUnprotectedTurnLeft();
  void OnHandleStateProtectedTurnLeft();
  void OnHandleStateUTurn();
  bool DetectStraight();
  bool DetectTurnRight();
  bool DetectLeftWaitZone();
  bool DetectTurnLeft();
  bool DetectUTurn();
  bool DetectUnprotectedTurnLeft();
  bool DetectProtectedTurnLeft();
  bool DetectExit();
  RefLineType GetRefLineType(double length);
  RefLineType GetRefLineType();
  bool HaveSignal();

  const config::AutoPlanConfig::MotorwayIntersectionScenario&
      motor_intersection_scenario_ = config::PlanningConfig::Instance()
                                         ->plan_config()
                                         .motorway_intersection_scenario;
  RefLineType ref_line_type_;
  double uturn_width_{10.0};
  bool have_signal_{false};
  static constexpr int kFilterFrameThreshold = 3;
  neodrive::global::planning::MotorwayIntersectionStageState::State curr_state_;
  neodrive::global::planning::MotorwayIntersectionStageState::State prev_state_;
  neodrive::global::planning::MotorwayIntersectionStageChangeFlag::ChangeFlag
      prev_change_flag_;
  neodrive::global::planning::MotorwayIntersectionStageChangeFlag::ChangeFlag
      curr_change_flag_;
  neodrive::global::planning::MotorwayIntersectionStageChangeFlag::ChangeFlag
      filter_change_flag_;
  std::unordered_map<MotorwayIntersectionStageState::State,
                     void (ScenarioMotorwayIntersectionDecider::*)()>
      state_machine_function_map_;
  const neodrive::common::config::AutoDriveStrategy::MotorWay&
      motorway_config_ = common::config::CommonConfig::Instance()
                             ->drive_strategy_config()
                             .motor_way;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioMotorwayIntersectionDecider);

}  // namespace planning
}  // namespace neodrive
