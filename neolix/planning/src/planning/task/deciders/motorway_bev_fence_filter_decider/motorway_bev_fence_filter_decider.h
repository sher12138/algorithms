#pragma once

#include <unordered_set>

#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/internal_data_handle.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {
namespace {
using MatrixInterval = util::internal_data_handle::MatrixInterval;
using VecInterval = util::internal_data_handle::VecInterval;
}  // namespace
class MotorwayBevFenceFilterDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwayBevFenceFilterDecider);

 public:
  virtual ~MotorwayBevFenceFilterDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);
  bool EgoBePortectedByFence(double s, double speed,
                             std::vector<MatrixInterval>& combine_data);
  bool Process(TaskInfo& task_info);
  void IgnoreOBsByPerceptionRightFence(TaskInfo& task_info);
  bool ObsBePortectedByFence(const std::vector<MatrixInterval>& combine_data,
                             Obstacle* const& obstacle,
                             const bool ego_be_protected_by_right_fence);
};
REGISTER_SCENARIO_TASK(MotorwayBevFenceFilterDecider);
}  // namespace planning
}  // namespace neodrive
