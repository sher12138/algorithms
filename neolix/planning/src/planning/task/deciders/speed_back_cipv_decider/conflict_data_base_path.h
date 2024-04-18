#pragma once

#include "conflict_data_interface.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/config/auto_planning_research_config.h"
#include "src/planning/config/planning_config.h"

namespace neodrive {
namespace planning {

class BackDataBasePath final : public BackDataInterface {
 public:
  BackDataBasePath();
  virtual ~BackDataBasePath() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictMergeInData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictMeetingData(
      TaskInfo& task_info) override;
  virtual std::vector<ConnectionConflictInfo> ComputeConflictCustomData(
      TaskInfo& task_info) override;
  bool CheckCollisionForDiverging(TaskInfo& task_info,
                                  const Obstacle& obstacle);

 private:
  bool CollisionCheckWithObstaclePolyline(
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data,
      std::vector<double>* conflict_area_bound);

  void GetObsPredictionSegment(const InsidePlannerData& inside_data,
                               const Obstacle& obstacle, OBS_SIDE& obs_side,
                               Segment2d& obs_prediction_segment,
                               Vec2d& obs_base_pt);

  void GetFirstCollideInfo(
      const OutsidePlannerData* const outside_data,
      const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
      AdcCollideCornerPoint& adc_first_collide_corner_point,
      AdcCollideCornerPoint& ego_collide_corner_point,
      AdcCollideCornerPoint& agent_collide_corner_point,
      int& path_first_collide_i, Vec2d& path_first_collide_pt,
      int& path_ego_end_collide_i, Vec2d& path_ego_end_collide_pt,
      int& path_agent_end_collide_i, Vec2d& path_agent_end_collide_pt);

  bool GetLaneMergingInfo(const std::vector<PathPoint>& path_points,
                          const int path_first_collide_i, const int obs_id,
                          const Vec2d& path_first_collide_pt,
                          const int ego_end_collide_i,
                          const int agent_end_collide_i,
                          const Vec2d& agent_end_collide_pt,
                          const double agent_length, const Vec2d& obs_base_pt,
                          std::vector<double>* conflict_area_bound);

  void GetAdcCornerPointCoordinate(
      const Vec2d& adc_coordinate, const double adc_heading,
      std::vector<Vec2d>& adc_corner_pt_coordinate);

  void GetEmittedFromObsCenter(const Obstacle& obstacle,
                               Segment2d& obs_prediction_segment,
                               Vec2d& obs_base_pt);

  bool CheckFirstCollide(const OutsidePlannerData* const outside_data,
                         const Segment2d& obs_prediction_segment);

 private:
  const config::AutoPlanningResearchConfig::
      MotorwaySpeedConfilictDeciderConfig::ConflictData::BasePath&
          conflict_data_base_path_config_ =
              config::PlanningConfig::Instance()
                  ->planning_research_config()
                  .motorway_speed_confilict_decider_config.conflict_data
                  .base_path;
};

}  // namespace planning
}  // namespace neodrive
