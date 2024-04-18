#pragma once

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/internal_data_handle.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {
enum JudgeObsInSamePos { kOnLine, kLeft, kRight };

namespace {
using MatrixInterval = util::internal_data_handle::MatrixInterval;
using VecInterval = util::internal_data_handle::VecInterval;
}  // namespace

class FenceFilterReverseOBs {
 public:
  static void IgnoreOBsByPerceptionRightFence(TaskInfo& task_info);

 private:
  static bool EgoBePortectedByFence(double, double,
                                    std::vector<MatrixInterval>&);
  static bool ObsBePortectedByFence(
      const std::vector<MatrixInterval>& combine_data,
      Obstacle* const& obstacle, const bool ego_be_protected_by_right_fence);
};

class MotorwaySpeedPredictionPreDecisionDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedPredictionPreDecisionDecider);

 public:
  ErrorCode Execute(TaskInfo& task_info) override;
  void IgnoreOBsByPerceptionFence(TaskInfo& task_info);
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  void IgnoreObsByPerceptionRoadBound(TaskInfo& task_info);
  void VisCerbBoundary(const ReferenceLinePtr ref_line,
                       const std::vector<Boundary>& check_area,
                       const std::string event_name);
  bool JudgeEgoInJunction(TaskInfo& task_info, const double& ego_s);
  void GetObsOnPath(TaskInfo& task_info);

 private:
  std::unordered_set<int> obs_not_ignore_{};
  std::vector<Boundary> curb_area_;
};

REGISTER_SCENARIO_TASK(MotorwaySpeedPredictionPreDecisionDecider);

}  // namespace planning
}  // namespace neodrive
