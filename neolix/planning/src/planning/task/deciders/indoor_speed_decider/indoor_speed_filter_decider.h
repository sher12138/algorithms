#pragma once

#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class IndoorSpeedFilterDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(IndoorSpeedFilterDecider);

 public:
  virtual ~IndoorSpeedFilterDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  std::vector<Vec2d> GetLidarPoints(
      const neodrive::global::perception::Freespace& freespace) const;
  double SegmentLengthInsidePolygon(const Polygon2d& polygon,
                                    const Segment2d& segment) const;
};

REGISTER_SCENARIO_TASK(IndoorSpeedFilterDecider);

}  // namespace planning
}  // namespace neodrive