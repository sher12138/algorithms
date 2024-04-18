#pragma once
#include <string>
#include <unordered_map>

#include "common/visualizer_event/visualizer_event.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/sim_planner/sim_lateral_track_manager.h"
#include "src/planning/sim_planner/sim_map.h"
#include "src/planning/sim_planner/sim_task.h"

namespace neodrive {
namespace planning {
std::unordered_map<std::string, int> all_lanes_id = {
    {"left", 0}, {"center", 0}, {"right", 0}};
std::vector<sim_planner::SimMapPoint> track_pts;

class SimPlannerTrackOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SimPlannerTrackOptimizer);

 public:
  virtual ~SimPlannerTrackOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};
};

REGISTER_SCENARIO_TASK(SimPlannerTrackOptimizer);

}  // namespace planning
}  // namespace neodrive