#pragma once

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
struct ReverseObs {
  int obs_id{0};
  MotorwayMultiCipvSpeedObstacleDecision reverse_obstacle_decision;

  double obs_longitudinal_dis{0.0};
  bool if_ignore{false};
  double heading_diff{0.0};
  void Reset() {
    obs_id = 0;
    if_ignore = false;
    obs_longitudinal_dis = 0.0;
    heading_diff = 0.0;
  }
};

class MotorwaySpeedProcessReverseDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedProcessReverseDecider);

 public:
  virtual ~MotorwaySpeedProcessReverseDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  bool JudgeObsInJunction(TaskInfo& task_info, const double& obs_s);

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  void reset();
  void CalProcessAction(TaskInfo& task_info);

 private:
  std::vector<ReverseObs> reverse_obs_list_;

  double adc_front_edge_s_{0.0};
  double adc_current_s_{0.0};
  double adc_current_v_{0.0};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedProcessReverseDecider);

}  // namespace planning
}  // namespace neodrive
