#pragma once

#include <string>

#include "common/visualizer_event/visualizer_event.h"
#include "region_decision_graph_search.h"
#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/sim_planner/sim_lateral_manager.h"
#include "src/planning/sim_planner/sim_map.h"
#include "src/planning/sim_planner/sim_task.h"

namespace neodrive {
namespace planning {

sim_planner::SimLateralManager::ReplanningContext context;
sim_planner::SimLateralManager::Snapshot last_snapshot;
sim_planner::Task last_task;
sim_planner::SimLateralManager::LaneChangeContext lc_context;
sim_planner::SimLateralManager::LaneChangeProposal last_lc_proposal;
std::vector<sim_planner::SimLateralManager::ActivateLaneChangeRequest>
    preliminary_active_requests;

class SimPlannerRegionSearchOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SimPlannerRegionSearchOptimizer);

 public:
  virtual ~SimPlannerRegionSearchOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool ComputeTunnelWidth(const std::vector<PathRegion::Bound>& bounds_info,
                          OutsidePlannerData* const outside_data);

  LaneBorrowScenario StateIdentify(
      const InsidePlannerData& inside_data,
      const std::vector<PathRegion::Bound>& bounds_info,
      const SLPoint& init_sl_point, const bool is_lane_borrow,
      OutsidePlannerData* const outside_data);
};

REGISTER_SCENARIO_TASK(SimPlannerRegionSearchOptimizer);

}  // namespace planning
}  // namespace neodrive
