#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class PathAttentionRegionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathAttentionRegionDecider);

 public:
  virtual ~PathAttentionRegionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool ComputeAttentionRegion(TaskInfo& task_info);

  void VisObstacles(std::vector<Obstacle>& obstacle, const std::string& name);
};

REGISTER_SCENARIO_TASK(PathAttentionRegionDecider);

}  // namespace planning
}  // namespace neodrive