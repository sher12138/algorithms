#pragma once

#include "planning/common/data_center/data_center.h"
#include "planning/common/visualizer_event/visualizer_event.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"
// #include "src/planning/navigation/common/navigation_swap_context.h"

namespace neodrive {
namespace planning {
class ScenarioMotorwayLaneChangeDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioMotorwayLaneChangeDecider);

  using MotorwayLaneChangeStageState =
      neodrive::global::planning::MotorwayLaneChangeStageState;
  using MotorwayLaneChangeStageChangeFlag =
      neodrive::global::planning::MotorwayLaneChangeStageChangeFlag;

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 public:
  inline const MotorwayLaneChangeStageState::State& curr_state() {
    return data_center_->mutable_master_info()
        ->motorway_lane_change_context()
        .stage;
  }
  inline void set_curr_state(const MotorwayLaneChangeStageState::State& state) {
    data_center_->mutable_master_info()
        ->mutable_motorway_lane_change_context()
        ->stage = state;
  }

 private:
  void RegisterStateMachineResponseFunctions();
  bool ResetStateMachine();
  bool UpdateStateMachine();

  void OnHandleStatePrepare();
  void OnHandleStateWaiting();
  void OnHandleStateChanging();
  void OnHandleStateFinish();
  void OnHandleStateCancel();

  void PrepareMotorwayLaneChangeCondition();
  void BuildAdcBoundary(const ReferenceLinePtr& reference_line,
                        const TrajectoryPoint& init_point,
                        Boundary& adc_boundary) const;
  bool ExtendLaneBound();
  void CreateLaneChangeVirtualObs();
  bool GetTargetL();
  void LimitRefLByObs();
  void ComputeEndDis();
  void ComputeRemainTime();
  bool IsTargetLaneDynamicObsClear();
  bool IsTargetLaneStaticObsClear();
  bool IsFrontHasLaneTurn() const;
  bool IsFrontHasRoadBoundary() const;
  bool IsFrontHasDividerRestrict() const;
  bool IsOnCurrentLane() const;
  bool IsOverCurrentLane() const;
  bool IsOnTargetLane() const;
  bool IsFrontHasTrafficLight() const;

  bool CheckWaitingTriggered();
  bool CheckChangingTriggered();
  bool CheckFinishTriggered();
  bool CheckCancelTriggered();

  bool CheckPrepareFinished();
  bool CheckWaitingFinished();
  bool CheckChangingFinished();
  bool CheckFinishFinished();
  bool CheckCancelFinished();

  void SaveMonitorMessage();

 private:
  static constexpr int kFilterFrameThreshold = 10;
  static constexpr int kExtendFrontDis = 200;
  static constexpr int kExtendBackDis = 10;
  MotorwayLaneChangeStageChangeFlag::ChangeFlag prev_decision_{
      MotorwayLaneChangeStageChangeFlag::T_NONE_PREPARE};
  MotorwayLaneChangeStageChangeFlag::ChangeFlag curr_decision_{
      MotorwayLaneChangeStageChangeFlag::T_NONE_PREPARE};
  MotorwayLaneChangeStageChangeFlag::ChangeFlag filter_decision_{
      MotorwayLaneChangeStageChangeFlag::T_NONE_PREPARE};
  const config::AutoPlanConfig* plan_config_ptr_{nullptr};
  std::unordered_map<MotorwayLaneChangeStageState::State,
                     void (ScenarioMotorwayLaneChangeDecider::*)()>
      state_machine_function_map_;
  SLPoint target_point_;

  std::vector<Obstacle> near_obstacles_{};
  std::vector<Obstacle> far_obstacles_{};
  double veh_v_ = 0.0;
  double remain_lat_dis_ = 0.0;
  double remain_time_ = 0.0;
  double dis_to_current_ref_end_ = 0.0;
  double dis_to_change_end_ = 0.0;
  double change_need_dis_ = 0.0;
  ReferencePoint change_end_point_;

  double veh_ratio_l_{};
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioMotorwayLaneChangeDecider);

}  // namespace planning
}  // namespace neodrive
