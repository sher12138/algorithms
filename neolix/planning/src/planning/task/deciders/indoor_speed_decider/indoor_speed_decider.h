#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class IndoorSpeedDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(IndoorSpeedDecider);

 public:
  virtual ~IndoorSpeedDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  std::vector<Vec2d> GetLidarPoints(
      const neodrive::global::perception::Freespace& freespace) const;

 private:
  double resolution_{0.05};
  double time_length_{8.0};
  double time_step_{0.1};
  std::size_t size_post_data_{81};
};

REGISTER_SCENARIO_TASK(IndoorSpeedDecider);

}  // namespace planning
}  // namespace neodrive