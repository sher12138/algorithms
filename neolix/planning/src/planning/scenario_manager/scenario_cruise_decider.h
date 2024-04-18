#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioCruiseDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioCruiseDecider);

  using CruiseStageState = neodrive::global::planning::CruiseStageState;
  using CruiseStageChangeFlag =
      neodrive::global::planning::CruiseStageChangeFlag;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 public:
  inline const CruiseStageState::State& curr_state() {
    return data_center_->mutable_master_info()->cruise_context().stage;
  }
  inline void set_curr_state(const CruiseStageState::State& state) {
    data_center_->mutable_master_info()->mutable_cruise_context()->stage =
        state;
  }

 private:
  void RegisterStateMachineResponseFunctions();
  bool ResetStateMachine();
  bool UpdateStateMachine();

  void OnHandleStateKeep();
  void OnHandleStateCentering();
  void OnHandleStateBiasDrive();
  void OnHandleStateRightAvoid();
  void OnHandleStateLeftOvertake();
  void OnHandleStatePullOver();

  bool CheckKeepFinished();
  bool CheckCenteringTriggered();
  bool CheckCenteringFinished();
  bool CheckBiasDriveTriggered();
  bool CheckBiasDriveFinished();
  bool CheckRightAvoidTriggered();
  bool CheckRightAvoidFinished();
  bool CheckLeftOvertakeTriggered();
  bool CheckLeftOvertakeFinished();
  bool CheckPullOverTriggered();
  bool CheckPullOverFinished();

  bool IsFarway(const Obstacle& obstacle, const Boundary& adc_boundary);
  bool IsInbound(
      const Obstacle& obstacle,
      const std::vector<neodrive::planning::PathRegion::Bound>& bounds,
      double& obs_left_witdh, double& obs_right_witdh);
  bool ComputeAttentionInfos();
  bool AllowPullOverInTurn(const TaskInfo& task_info);
  void SaveMonitorMessage();

 private:
  static constexpr int kFilterFrameThreshold = 3;
  static constexpr double kLaneTurnKappaThreshold = 0.01;

  CruiseStageChangeFlag::ChangeFlag prev_decision_{
      CruiseStageChangeFlag::T0_NONE_KEEP};
  CruiseStageChangeFlag::ChangeFlag curr_decision_{
      CruiseStageChangeFlag::T0_NONE_KEEP};
  CruiseStageChangeFlag::ChangeFlag filter_decision_{
      CruiseStageChangeFlag::T0_NONE_KEEP};
  std::unordered_map<CruiseStageState::State, void (ScenarioCruiseDecider::*)()>
      state_machine_function_map_;
  std::vector<Obstacle> near_obstacles_{};
  std::vector<Obstacle> far_obstacles_{};
  bool only_front_slow_obs_ = false;
  bool only_near_right_front_obs_ = true;
  double front_slow_obs_max_l_ = 0.0;
  double front_slow_obs_min_l_ = 0.0;
  double near_front_slow_obs_max_l_ = 0.0;
  double target_s_ = 0.0;
  double veh_v_ = 0.0;
  bool is_front_turn_ = false;
  bool is_in_junction_ = false;

  const neodrive::common::config::AutoDriveStrategy::NonMotorway&
      non_motorway_config_ = common::config::CommonConfig::Instance()
                                 ->drive_strategy_config()
                                 .non_motorway;
  const neodrive::planning::config::AutoPlanConfig::CruiseScenario&
      cruise_scenario_conf_ =
          config::PlanningConfig::Instance()->plan_config().cruise_scenario;
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioCruiseDecider);

}  // namespace planning
}  // namespace neodrive
