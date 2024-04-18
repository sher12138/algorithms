#pragma once

#include "src/planning/config/planning_config.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedLaneBorrowDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedLaneBorrowDecider);

 public:
  virtual ~SpeedLaneBorrowDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

 private:
  bool PathCrossLaneBound(TaskInfo& task_info);
  bool GetCollisionInfoFromCollisionCheck(
      TaskInfo& task_info, const Obstacle* const obs, bool reverse,
      double project_vel,
      std::vector<LaneBorrowPrepareCollisionInfo>& collision_info);
  bool DynamicObsCheck(TaskInfo& task_info,
                       const std::vector<Obstacle*>& dynamic_obs_vec);

  bool BorrowRegionsCheck(TaskInfo& task_info, const Obstacle* obs) const;

  bool IsDynamicObsNeedIgnord(TaskInfo& task_info, const Obstacle* obs,
                              bool& reverse, double& project_v);

  bool CalcBorrowBoundsInfo(TaskInfo& task_info);

  bool BorrowRegionCollideCheck(const Obstacle* const obs,
                                const double predict_time);

  bool PrepareStatus(TaskInfo& task_info);

  bool STDecision(TaskInfo& task_info);

  std::size_t PathIndex(TaskInfo& task_info, const double s);

  bool CreateVirtualObstacle(TaskInfo& task_info);

  bool IgnoreObsByObsHeading(TaskInfo& task_info, const Obstacle* obs);

  bool IgnoreObsByObsSpeed(TaskInfo& task_info, const Obstacle* obs);

  bool IgnoreObsByLateralDistance(TaskInfo& task_info, const Obstacle* obs);

  bool IgnoreObsByBackBoundary(TaskInfo& task_info, const Obstacle* obs);

  bool CalPathZeroKappaEndS(TaskInfo& task_info, double& expand_s);

 private:
  double adc_current_v_{0.0};
  double adc_front_edge_s_{0.0};
  SLPoint init_sl_point_;
  Vec2d path_cross_lane_point_{};
  double path_cross_lane_s_{0.0};
  double walk_s_befor_stop_{
      std::numeric_limits<double>::infinity()};  // used to limit speed
  double stop_s_in_frenet_{
      std::numeric_limits<double>::infinity()};  // used to create virtual obs
  int final_obs_id_{-1};
  std::vector<double> borrow_attention_s_vec_{};
  std::vector<double> borrow_attention_lane_bounds_{};
  std::vector<double> borrow_attention_bounds_{};
  std::vector<Segment2d> borrow_lane_bound_segments_{};
  std::vector<Polygon2d> borrow_regions_{};
  Boundary back_attention_boundary_{};
  config::AutoPlanningResearchConfig::LaneBorrowSpeedDeciderConfig config_{};
  bool stop_flag_{false};
  double limited_speed_{0.0};
  bool update_limited_speed_{false};
  double last_limited_speed_{0.0};
  bool has_collision_risk{false};
};

REGISTER_SCENARIO_TASK(SpeedLaneBorrowDecider);

}  // namespace planning
}  // namespace neodrive
