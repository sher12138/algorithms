#pragma once
#include "planning/planning_map/planning_map.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/speed_constrained_iter_lqr_planner/speed_cilqr_model.h"
#include "src/planning/task/deciders/speed_constrained_iter_lqr_planner/speed_cilqr_obs.h"
#include "src/planning/task/deciders/speed_constrained_iter_lqr_planner/speed_cilqr_solver.h"
namespace neodrive {
namespace planning {

class SpeedConstrainedIterLqrDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedConstrainedIterLqrDecider);

 public:
  virtual ~SpeedConstrainedIterLqrDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  void SaveLOGResults();
  bool ProcessObs(TaskInfo& task_info);
  void CalInitStateAndControl(const InsidePlannerData& inside_data);

 private:
  bool Init(TaskInfo& task_info);
  void reset();
  bool Process(TaskInfo& task_info);

 private:
  CilqrModel::CilqrState ego_state_;
  RefSpeedPlan ref_speed_plan_;
  std::size_t n_{0};
  std::vector<std::shared_ptr<SpeedCilqrObsProcess>> sorted_obs_list_{};
  double delta_t_{0.1};
  bool task_failed{false};
};
REGISTER_SCENARIO_TASK(SpeedConstrainedIterLqrDecider);
}  // namespace planning
}  // namespace neodrive