#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedSlowDownDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedSlowDownDecider);

 public:
  virtual ~MotorwaySpeedSlowDownDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool SlowDown(TaskInfo& task_info);

 private:
  TrajectoryPoint real_pos_point_{};
  bool lms_sensor_check_failed_{false};
  bool road_bound_safe_check_failed_failed_{false};
  bool curb_risk_slow_down_{false};
  bool road_bound_risk_slow_down_{false};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedSlowDownDecider);

}  // namespace planning
}  // namespace neodrive
