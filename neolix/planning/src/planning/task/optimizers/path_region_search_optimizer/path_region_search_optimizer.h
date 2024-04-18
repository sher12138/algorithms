#pragma once

#include <string>

#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class PathRegionSearchOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathRegionSearchOptimizer);

 public:
  virtual ~PathRegionSearchOptimizer() override;

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

  bool LowSpeedPolygonPreProcess(
      const PathObstacleDecision& obs_decision_element,
      const ReferenceLinePtr& reference_line, double front_enlarge_s,
      double back_enlarge_s, double dt,
      std::vector<std::vector<Vec2d>>& low_speed_polygon_vec,
      std::vector<std::vector<Vec2d>>& low_speed_polygon_transfer_vec,
      std::vector<Vec2d>& aabox2d_points);

  bool ObsNearPolygonPreProcess(
      const PathObstacleDecision& obs_decision_element,
      const ReferenceLinePtr& reference_line, const Boundary& boundary,
      double front_enlarge_s, double back_enlarge_s, double ref_s,
      std::vector<std::vector<Vec2d>>& near_ego_polygon_vec,
      std::vector<std::vector<Vec2d>>& near_ego_polygon_transfer_vec,
      std::vector<Vec2d>& aabox2d_points);

  void BarrierGatePolygonPreProcess(double adc_end_s,
                                    const ReferenceLinePtr& reference_line,
                                    std::vector<Vec2d>& aabox2d_points);
  std::vector<AABox2d> CreatVirtualObs(TaskInfo& task_info,
                                       const double& steering,
                                       const double& velocity,
                                       const double& heading);
};

REGISTER_SCENARIO_TASK(PathRegionSearchOptimizer);

}  // namespace planning
}  // namespace neodrive
