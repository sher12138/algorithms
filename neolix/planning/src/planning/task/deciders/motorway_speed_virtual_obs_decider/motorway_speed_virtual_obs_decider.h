#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedVirtualObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedVirtualObsDecider);

 public:
  virtual ~MotorwaySpeedVirtualObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

 private:
  bool ReferenceLineEndVirtualObs(TaskInfo& task_info) const;

  bool RoutingDestinationVirtualObs(TaskInfo& task_info) const;

  bool StationModeVirtualObs(TaskInfo& task_info) const;

  bool TrafficLawVirtualObs(TaskInfo& task_info) const;

 private:
  bool is_cruise_{false};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedVirtualObsDecider);

}  // namespace planning
}  // namespace neodrive