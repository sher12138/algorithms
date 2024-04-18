#pragma once

#include "planning.pb.h"
#include "planning/common/data_center/decision_context.h"
#include "planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class VehicleStateControlDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(VehicleStateControlDecider);

 public:
  virtual ~VehicleStateControlDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  enum class TurnLightSignal {
    NO_TURN = 1,
    LEFT_TURN = 2,
    RIGHT_TURN = 3,
    LEFT_CHANGING_LANE = 4,
    RIGHT_CHANGING_LANE = 5,
    LEFT_U_TURN = 6,
    RIGHT_U_TURN = 7,
    LEFT_PULL_OVER = 8,
    RIGHT_PULL_OVER = 9,
    REVERSE_TURN = 10,
    NUDGE_OBS_LEFT_TURN = 11,
    NUDGE_OBS_RIGHT_TURN = 12,
    NUDGE_OBS_LEFT_TURN_BACK = 13,  // turn right
    NUDGE_OBS_RIGHT_TURN_BACK = 14  // turn left
  };

  AutoPilotIntention GetNewIntention();
  void FilterIntention(const AutoPilotIntention new_intention);
  void UpdateIntentionTurnLight();

 private:
  TurnLightSignal last_signal_;
  double last_signal_start_time_;
  double prev_intention_update_timestamp_{0.0};
  AutoPilotIntention curr_intention_{AutoPilotIntention::FINISH};
  bool clean_adc_signal_{false};
  ADCSignals::SignalType adc_signal_{ADCSignals::LEFT_TURN};
};

REGISTER_SCENARIO_TASK(VehicleStateControlDecider);

}  // namespace planning
}  // namespace neodrive
