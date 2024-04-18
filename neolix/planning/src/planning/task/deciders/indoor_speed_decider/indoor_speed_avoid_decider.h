#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class IndoorSpeedAvoidDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(IndoorSpeedAvoidDecider);

 public:
  virtual ~IndoorSpeedAvoidDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
 private:
  double resolution_{0.05};
  double time_length_{8.0};
};

REGISTER_SCENARIO_TASK(IndoorSpeedAvoidDecider);

}  // namespace planning
}  // namespace neodrive