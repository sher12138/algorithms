#pragma once

#include "common/util/time_logger.h"
#include "global_adc_status.pb.h"
#include "pilot_state_msgs.pb.h"
#include "planning/config/planning_config.h"
#include "proto/localization_status.pb.h"
#include "proto/patrol_result.pb.h"
#include "proto/world_model.pb.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "state_machine/state_machine.h"

namespace neodrive {
namespace planning {

class PilotStateDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PilotStateDecider);

  using StateMachineMap = std::map<std::string, void (PilotStateDecider::*)()>;
  using StateMachineMapPtr = std::shared_ptr<StateMachineMap>;
  using StateMachine = neodrive::common::state_machine::StateMachine;
  using StateMachineShrPtr = std::shared_ptr<StateMachine>;
  using ErrLevel = neodrive::global::patrol::ErrLevel;
  using ErrResultItem = neodrive::global::patrol::ErrResultItem;

  enum class PilotErrorLevel { NORMAL = 0, ALARM = 1, SERIOUS = 2 };

 public:
  virtual ~PilotStateDecider() override;

  void UpdatePilotState();

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override;
  void Reset() override;

  DEFINE_SIMPLE_TYPE_GET_FUNCTION(std::string, current_state);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int8_t, prev_decision);
  DEFINE_SIMPLE_TYPE_GET_FUNCTION(int8_t, current_decision);

 private:
  bool Init();
  void UpdateStateMachine();

  /**
   * @brief handle emergency event which may happen in all states
   */
  bool OnHandleAllStates();
  /**
   * @brief handle event in each state
   */
  void OnHandleStateStandby();
  void OnHandleStateReject();
  void OnHandleStateActive();
  void OnHandleStateDegradation();
  void OnHandleStateEscalation();
  /**
   * @brief register state machine response functions
   */
  void RegisterStateMachineResponseFunctions();
  void CheckAndSetFlag();
  void SmoothSpeedLimitAction();
  void PackagePilotStateMsg();
  void AddTakeoverEvent();
  global::planning::PilotChangeFlag::ChangeFlag BackToNomalState(bool);
  inline PilotErrorLevel GetPilotErrorLevelFromErrItem(
      const ErrResultItem &erritem) {
    switch (erritem.err_level()) {
      case ErrLevel::LEVEL_WARNING:
        return PilotErrorLevel::NORMAL;
      case ErrLevel::LEVEL_FATAL:
        return PilotErrorLevel::ALARM;
      case ErrLevel::LEVEL_SERIOUS:
        return PilotErrorLevel::SERIOUS;
      default:
        break;
    }
  }

  /**
   * monitor str msg for debug.
   */
  void SaveMonitorMessage();

 private:
  static constexpr double kFrequentDegradationTimeThreshold = 60.0;

 private:
  const config::AutoPlanConfig *plan_config_ptr_;
  std::shared_ptr<
      neodrive::cyber::Writer<neodrive::global::planning::PilotState>>
      pilot_state_pub_;

  // map of handle state function
  StateMachineMapPtr handle_state_function_map_{
      std::make_shared<StateMachineMap>()};
  // state machine
  StateMachineShrPtr state_machine_{std::make_shared<StateMachine>()};
  // current state (from state machine)
  std::string current_state_{""};
  std::string prev_state_{""};
  std::string reason_{""};
  // previouse planning decision
  int8_t prev_decision_{0};
  // current decision: new change flag for state machine
  int8_t current_decision_{0};

  bool vehicle_state_valid_{false};
  bool is_permanent_error_{false};
  double latest_patrol_error_time_{0.0};
  double last_upgradation_time_{0.0};
  double last_degradation_time_{0.0};
  PilotErrorLevel pilot_error_level_{PilotErrorLevel::NORMAL};

  bool is_state_changed_;
  bool is_emergency_braking_{false};
  bool prev_is_auto_driving_{false};
  uint32_t sequence_num_{0};
  bool update_limited_speed_{false};
  double last_lidar_perception_error_t_{-1.0};
  double limited_speed_;
  double sys_start_t_;
  std::queue<double> lidar_freespace_empty_q_;

  std::string prev_error_status_{""};
  double prev_error_status_time_{0};
  std::string machine_type_{"A"};
  std::string drive_strategy_{"beijing"};

  int wheel_error_cnt_{0};
  bool is_wheel_error_{false};

  std::shared_ptr<PoseStamped> prev_utm_msg_{nullptr};
  std::shared_ptr<PoseStamped> curr_utm_msg_{nullptr};
  bool is_localization_pos_jump_{false};

  bool is_need_takeover_{false};
  double block_start_timestamp_{0.0};
};

REGISTER_SCENARIO_TASK(PilotStateDecider);

}  // namespace planning
}  // namespace neodrive
