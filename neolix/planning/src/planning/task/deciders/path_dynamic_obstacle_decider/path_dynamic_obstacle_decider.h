/// @file Define the class to consider moving obstacles
#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

/// @namespace neodrive::planning
namespace neodrive {
namespace planning {

/// @class Decide which dynamic obstacle to consider
class PathDynamicObstacleDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathDynamicObstacleDecider);

 public:
  virtual ~PathDynamicObstacleDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(PathDynamicObstacleDecider);

}  // namespace planning
}  // namespace neodrive
