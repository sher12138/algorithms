#pragma once

#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class PathObsPreDecisionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathObsPreDecisionDecider);

 public:
  virtual ~PathObsPreDecisionDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(const ReferenceLinePtr& reference_line,
            const InsidePlannerData& inside_data,
            const DecisionData& decision_data,
            OutsidePlannerData* const outside_data);

  bool Process(const ReferenceLinePtr& reference_line,
               const InsidePlannerData& inside_data,
               OutsidePlannerData* const outside_data);

 private:
  void BuildAdcBoundary(const ReferenceLinePtr& reference_line,
                        const TrajectoryPoint& init_point,
                        Boundary& adc_boundary) const;

  bool CalcLaneBound(const ReferenceLinePtr& reference_line, double start_s,
                     double end_s, double offset, double* left_bound,
                     double* right_bound, double* bound_width) const;

  bool GetObstaclesBoundary(
      const ReferenceLinePtr& reference_line, const DecisionData& decision_data,
      std::vector<PathObstacleBoundary>* obstacles_boundary) const;

  bool CalcMaxLaneBound(const ReferenceLinePtr& reference_line,
                        double origin_start_s, double origin_end_s,
                        double offset, double* left_bound,
                        double* right_bound) const;

  bool VirtualObstacleNeedIgnore(const Obstacle& virtual_obstacle) const;

  PathObstacleDecision CreateObstacleDecision(
      const Decision::DecisionType& decision_type,
      const PathObstacleBoundary& obstacle_boundary, const Box2d& obstacle_box,
      const double l_bound) const;

  PathObstacleDecision CreateObstacleDecision(
      const Boundary& adc_boundary, const double veh_x, const double veh_y,
      const double veh_theta, const double delta_theta,
      const Decision::DecisionType& decision_type,
      const PathObstacleBoundary& obstacle_boundary, const Box2d& obstacle_box,
      const double l_bound) const;

  bool MergeOverlapedObs(
      const InsidePlannerData& inside_data,
      std::vector<PathObstacleDecision>& obstacles_for_path) const;

  bool IgnoreVirtualObs(const PathObstacleBoundary& obs) const;

  bool IgnoreOutofLateralRange(
      const PathObstacleBoundary& obstacle_boundary) const;

  bool IgnoreOutofLongitudinalRange(
      const PathObstacleBoundary& obstacle_boundary,
      const Boundary& adc_boundary) const;

  bool IgnoreFastObs(const ReferenceLinePtr& reference_line,
                     const PathObstacleBoundary& obstacle_boundary,
                     const Boundary& adc_boundary, const double veh_v,
                     bool& safe_check) const;

  bool BackNearObs(const PathObstacleBoundary& obstacle_boundary,
                   const Boundary& adc_boundary,
                   std::vector<PathObstacleDecision>& obstacles_decision) const;

  bool LeftRightOnAdcObs(const PathObstacleBoundary& obstacle_boundary,
                         const Boundary& adc_boundary,
                         const Box2d& obstacle_box,
                         PathObstacleDecision& obstacle_decision) const;

  bool FrontObs(const ReferenceLinePtr& reference_line,
                const PathObstacleBoundary& obstacle_boundary,
                const Boundary& adc_boundary, const Box2d& obstacle_box,
                PathObstacleDecision& obstacle_decision, bool saft_obs) const;

  bool CalcObsHeadingDiff(const ReferenceLinePtr& reference_line,
                          const PathObstacleBoundary& obstacle_boundary,
                          double* heading_diff) const;

  bool IsSafeDynamicObs(const ReferenceLinePtr& reference_line,
                        const PathObstacleBoundary& obstacle_boundary,
                        const Boundary& adc_boundary, const double veh_v) const;

  std::size_t GetDynamicObsMotionRegion(const double heading_diff) const;

 private:
  double path_max_lateral_range_{0.};
  double left_lane_bound_{0.};
  double right_lane_bound_{0.};
  double min_lane_width_{0.};
  double min_corridor_width_{0.};
  double veh_x_{0.};
  double veh_y_{0.};
  double veh_theta_{0.};
  double delta_theta_{0.};
  SLPoint init_sl_point_;

  std::vector<PathObstacleBoundary> obstacles_boundary_{};

  double back_attention_dis_{0.};
  double back_obs_velo_thresh_{0.};
  double side_obs_velo_thresh_{0.};
  double front_reverse_obs_velo_thresh_{0.};
  double front_attention_dis_{0.};
  double front_side_pedestrain_dis_{0.};
  double left_max_lane_bound_{0.};
  double right_max_lane_bound_{0.};
  double fast_velo_{0.};
};

REGISTER_SCENARIO_TASK(PathObsPreDecisionDecider);

}  // namespace planning
}  // namespace neodrive
