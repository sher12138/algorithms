#pragma once

#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
enum RefLineType { STRAIGHT = 0, RIGHT = 1, LEFT = 2 };
class ScenarioIntersectionDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioIntersectionDecider);

  using IntersectionChangeFlag =
      neodrive::global::planning::IntersectionChangeFlag;
  using IntersectionState = neodrive::global::planning::IntersectionState;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

  inline const IntersectionState::State& curr_state() {
    return data_center_->mutable_master_info()->intersection_context().stage;
  }
  inline void set_curr_state(const IntersectionState::State& state) {
    data_center_->mutable_master_info()->mutable_intersection_context()->stage =
        state;
  }

 private:
  bool ResetStateMachine();
  void RegisterStateMachineResponseFunctions();
  bool UpdateStateMachine();
  void OnHandleStateInit();
  void OnHandleStateStraight();
  void OnHandleStateAboutToTurnRight();
  void OnHandleStateTurnRight();
  void OnHandleStateAboutToTurnLeft();
  void OnHandleStateTurnLeft();
  bool DetectStraight();
  bool DetectAboutToTurnRight();
  bool DetectTurnRight();
  bool DetectAboutToTurnLeft();
  bool DetectTurnLeft();
  void GetRefLineType();
  double GetKappa(double s);
  RefLineType CalRefLineType(double start_s, double length);
  const config::AutoPlanConfig::IntersectionScenario&
      intersection_scenario_config_ = config::PlanningConfig::Instance()
                                          ->plan_config()
                                          .intersection_scenario;
  double current_s_{0.0};
  RefLineType current_type_;
  RefLineType forward_type_;
  static constexpr int kFilterFrameThreshold = 3;
  neodrive::global::planning::IntersectionState::State curr_state_;
  neodrive::global::planning::IntersectionState::State prev_state_;
  neodrive::global::planning::IntersectionChangeFlag::ChangeFlag
      prev_change_flag_;
  neodrive::global::planning::IntersectionChangeFlag::ChangeFlag
      curr_change_flag_;
  neodrive::global::planning::IntersectionChangeFlag::ChangeFlag
      filter_change_flag_;
  std::unordered_map<IntersectionState::State,
                     void (ScenarioIntersectionDecider::*)()>
      state_machine_function_map_;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioIntersectionDecider);

}  // namespace planning
}  // namespace neodrive
