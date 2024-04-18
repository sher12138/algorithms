#pragma once

#include <memory>

#include "common/data_center/data_center.h"
#include "common/macros.h"
#include "cyber.h"
#include "cyber/proto/recorder_cmd.pb.h"
#include "global_adc_status.pb.h"
#include "pilot_state_msgs.pb.h"
#include "planning/common/boundary.h"
#include "planning/planning_map/planning_map.h"
#include "planning/scenario_manager/scenario_common.h"
#include "scenario_manager_msgs.pb.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/common/planning_types.h"
#include "src/planning/common/vehicle_param.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/reference_line/reference_line_util.h"
#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"
#include "src/planning/scenario_manager/scenario_task_pipeline.h"
#include "state_machine/state_machine.h"

namespace neodrive {
namespace planning {
class ScenarioManager {
  DECLARE_SINGLETON(ScenarioManager);

 public:
  ~ScenarioManager();
  const std::string& GetName() const { return name_; }
  bool Init();
  ErrorCode ExecuteBehaviorTasks(TaskInfo& task_info);
  ErrorCode ExecuteScenarioStageMotionTasks(TaskInfo& task_info);
  ScenarioTaskInterfacePtr GetDecider(const std::string& decider_name);
  void ResetDeciders();
  neodrive::global::planning::ScenarioState::State GetCurrentState() {
    return curr_state_;
  }

 private:
  bool LoadScenarioConfig();
  ErrorCode UpdateScenarioState();
  ErrorCode UpdateScenarioStage();
  ErrorCode ExecuteScenarioStageBehaviorTasks(TaskInfo& task_info);
  void SetStaticDetourVariables();
  void SetLanechangeSpeed();
  void UpdateLightTurn(TaskInfo &task_info);
  void RegisterStateMachineResponseFunctions();
  void MakeDecision();
  void UpdateStateMachine();

  void OnHandleStateInit();
  void OnHandleStateCruiseFollowing();
  void OnHandleStateDetour();
  void OnHandleStateNarrowRoadPass();
  void OnHandleStateIndoorCruise();
  void OnHandleStateParkCruise();
  void OnHandleStateParking();
  void OnHandleStateIntersectionPass();
  void OnHandleStateBackOutPass();
  void OnHandleStateBarrierGate();
  void OnHandleStateSideWayIntersectionPass();

  // motorway states
  void OnHandleMotorwayStateCruiseFollowing();
  void OnHandleMotorwayStateLaneChange();
  void OnHandleMotorwayStateIntersection();
  void OnHandleMotorwayStateDetour();

  bool CheckLaneChangeTriggered();
  bool CheckLaneChangeCondition();
  bool CheckLaneChangeCanceled();
  bool CheckLaneChangeFinished();
  bool CheckDetourTriggered();
  bool CheckDetourCondition();
  bool CheckDetourFinished() const;
  bool DetectNarrowRoad();
  bool CheckNarrowRoadPassFinished();
  bool DetectIndoorCruise();
  bool CheckIndoorCruiseFinished();
  bool DetectParkCruise();
  bool CheckParkCruiseFinished();
  bool DetectParking();
  bool CheckParkingFinished();
  bool DetectIntersection();
  bool DetectMotorwayCrossRoadJunction();
  bool DetectSideWayIntersection();
  bool DetectBackOut();
  bool CheckBackOutFinished();
  bool CheckIntersectionPassFinished();
  bool CheckIntersectionExportFinished();
  bool CheckBarrierGateTriggered();
  bool CheckBarrierGateCondition();
  bool CheckBarrierGateFinished();

  // motorway states
  bool CheckMotorwayCruiseTriggered();
  bool CheckMotorwayCruiseFinished();
  bool CheckMotorwayLaneChangeTriggered();
  bool CheckMotorwayLaneChangeFinished();
  bool CheckMotorwayIntersectionTriggered();
  bool CheckMotorwayIntersectionCondition();
  bool CheckMotorwayIntersectionFinished();
  bool CheckMotorwayDetourTriggered();
  bool CheckMotorwayDetourCondition();
  bool CheckMotorwayDetourFinished() const;

  bool CheckSideWayIntersectionPassFinished();
  void SaveMonitorMessage();

 private:
  bool CheckReroutingTriggered();
  bool CheckMotorwayReroutingTriggered();
  bool PrepareMotorwayLaneChangeCondition();
  bool PrepareDetourCondition();
  bool PrepareMotorwayDetourCondition();

 private:
  static constexpr int kFilterFrameThreshold = 3;
  static constexpr int kRefHeadingThreshold = 0.5;

 private:
  std::string name_{"ScenarioManager"};
  neodrive::common::state_machine::StateMachine state_machine_;
  std::unordered_map<neodrive::global::planning::ScenarioState::State,
                     void (ScenarioManager::*)()>
      state_machine_function_map_;
  bool initialized_{false};
  bool is_state_change_{false};
  bool state_change_finish_{false};
  neodrive::global::planning::ScenarioState::State curr_state_;
  std::string curr_state_str_;
  std::string prev_state_str_;
  neodrive::global::planning::ScenarioState::State prev_state_;
  neodrive::global::planning::ScenarioChangeFlag::ChangeFlag curr_change_flag_;
  neodrive::global::planning::ScenarioChangeFlag::ChangeFlag
      filter_change_flag_{
          neodrive::global::planning::ScenarioChangeFlag::T_INIT_CRUISE};
  bool lane_change_triggered_{false};
  bool lane_change_condition_satisfied_{false};
  bool lane_change_canceled_{false};
  bool lane_change_finished_{false};
  bool detour_triggered_{false};
  bool detour_condition_satisfied_{false};
  bool detour_finished_{false};
  double detour_obs_end_s_{0.0};
  bool narrow_road_detected_{false};
  bool narrow_road_pass_finished_{false};
  bool indoor_cruise_detected_{false};
  bool indoor_cruise_finished_{false};
  bool park_cruise_detected_{false};
  bool park_cruise_finished_{false};
  bool parking_detected_{false};
  bool parking_finished_{false};
  bool intersection_detected_{false};
  bool intersection_pass_finished_{false};
  bool intersection_export_finished_{false};
  bool side_way_intersection_detected_{false};
  bool side_way_intersection_finished_{false};
  bool side_way_intersection_export_finished_{false};
  bool back_out_detected_{false};
  bool back_out_finished_{false};
  bool barrier_gate_triggered_{false};
  bool barrier_gate_condition_satisfied_{false};
  bool barrier_gate_finished_{false};
  std::vector<Obstacle*> front_reverse_obs_{};
  std::vector<Obstacle*> left_front_reverse_obs_{}, right_front_reverse_obs_{};
  std::vector<Obstacle*> front_detour_obs_{};
  std::vector<Obstacle*> left_front_detour_obs_{}, right_front_detour_obs_{};
  std::string lane_type_;

  // motorway states
  bool motorway_cruise_triggered_{false};
  bool motorway_cruise_finished_{false};
  bool motorway_lane_change_triggered_{false};
  bool motorway_lane_change_finished_{false};
  bool motorway_intersection_triggered_{false};
  bool motorway_intersection_condition_satisfied_{false};
  bool motorway_intersection_finished_{false};
  bool motorway_intersection_export_finished_{false};
  bool motorway_detour_triggered_{false};
  bool motorway_detour_condition_satisfied_{false};
  bool motorway_detour_finished_{false};

  int count_back_out_{0};
  double last_freespace_time_{0.0};
  int freespace_fail_cnt_{0};
  int path_fail_cnt_{0};
  int speed_fail_cnt_{0};
  double veh_v_ = 0.0;
  uint64_t junction_id_{0};
  neodrive::planning::Vec2d last_back_out_point_{0.0, 0.0};

  std::map<std::string, ScenarioTaskInterfacePtr> all_deciders_;

  // key: {scenario_name}_{pipeline_name}. value: FunctionTaskPipelineShrPtr
  std::unordered_map<std::string, ScenarioStageDeciderInterfacePtr>
      scenario_decider_map_;
  std::unordered_map<std::string, ScenarioTaskPipelineShrPtr>
      scenario_task_pipeline_map_;
  ScenarioStageDeciderInterfacePtr curr_stage_decider_{nullptr};
  const config::AutoPlanConfig* plan_config_ptr_;
  DataCenter* data_center_{DataCenter::Instance()};
  std::unordered_map<std::string, std::vector<std::string>> common_tasks_map_;
};

}  // namespace planning
}  // namespace neodrive
