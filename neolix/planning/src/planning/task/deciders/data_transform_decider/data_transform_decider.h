#pragma once

#include "proto/scenario_manager_msgs.pb.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class DataTransformDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(DataTransformDecider);
  using DetourStageState = neodrive::global::planning::DetourStageState;
  using MotorwayDetourStageState =
      neodrive::global::planning::MotorwayDetourStageState;

 public:
  virtual ~DataTransformDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info) const;

  void BehaviorInfoTransform(TaskInfo& task_info) const;

  void VehicleStateTransform(TaskInfo& task_info) const;

  bool PlanningDataTransform(TaskInfo& task_info) const;

  void ProtectionDataTransform(TaskInfo& task_info) const;

  bool LmsSafetyCheck(TaskInfo& task_info) const;

 private:
};

REGISTER_SCENARIO_TASK(DataTransformDecider);

}  // namespace planning
}  // namespace neodrive
