#pragma once

#include <unordered_set>

#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_info_for_merge_in.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedConfilictMergeInDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedConfilictMergeInDecider);

 public:
  virtual ~MotorwaySpeedConfilictMergeInDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  void GenerateSafeCheckPolygon(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);
  void PosterioriUpdate(TaskInfo& task_info);
  bool EgoScenarioIsOk(TaskInfo& task_info);
  bool GetObsOnAttentionLane(TaskInfo& task_info);
  bool GetAttentionObsCollisionInfo(TaskInfo& task_info);
  void GetDecelerationByObs(TaskInfo& task_info, double& deceleration);
  void GetSpeedLimitByAttentionObs(TaskInfo& task_info);
  bool CollisionCheckWithObstaclePolyline(
      const ReferenceLinePtr reference_line,
      const InsidePlannerData& inside_data, const Obstacle& obstacle,
      OutsidePlannerData* const outside_data);
  void GetLaneMergingInfo(
      const std::vector<PathPoint>& path_points, const int obs_id,
      const AdcCollideCornerPoint& adc_first_collide_corner_point,
      const int path_first_collide_i, const Vec2d& path_first_collide_pt,
      const Vec2d& obs_base_pt);
  void GetFirstCollideInfo(
      const OutsidePlannerData* const outside_data,
      const Segment2d& obs_prediction_segment, const OBS_SIDE& obs_side,
      AdcCollideCornerPoint& adc_first_collide_corner_point,
      int& path_first_collide_i, Vec2d& path_first_collide_pt);
  void GetObsPredictionSegment(const InsidePlannerData& inside_data,
                               const Obstacle& obstacle, OBS_SIDE& obs_side,
                               Segment2d& obs_prediction_segment,
                               Vec2d& obs_base_pt);
  bool IgnoreObsOnCurrentLane(
      TaskInfo& task_info, const InsidePlannerData& inside_data,
      const TrafficConflictZoneContext& traffic_conflict_zone_context,
      const Obstacle* const obstacle);
  bool IgnoreObsOnVehicleFrame(const InsidePlannerData& inside_data,
                               const Obstacle* const obstacle);
  void GetAdcCornerPointCoordinate(
      const Vec2d& adc_coordinate, const double adc_heading,
      std::vector<Vec2d>& adc_corner_pt_coordinate);
  bool JudgeSameLane(Obstacle* const obstacle,
                     const ReferenceLinePtr& reference_line,
                     const double& adc_current_s);
  bool EgoLaneObsStraightIgnore(TaskInfo& task_info);
  bool CalcCollisionFree(TaskInfo& task_info);
  bool CalcSafeJudgement(
      const std::vector<ConnectionConflictInfo>& conflict_infos,
      const double v);

 private:
  const config::AutoPlanningResearchConfig::SpeedVehicleMergingDeciderConfig*
      speed_vehicle_merging_decider_config_ptr_{nullptr};

 private:
  double closest_vehcile_meeting_distance_{};
  double adc_current_v_{0.0};
  // all lane meetings on reference line after ego car.
  std::unordered_set<LaneMeetingRanges::ConstPtr> lane_meeting_ranges_{};
  // following four vectors: same obs, same index.
  double speed_limit_{0.0};
  double deceleration_{0.0};
  bool update_speed_limit_{false};

  double adc_start_l_{0.0};
  double adc_end_l_{0.0};
  double adc_width_{0.0};
  double adc_length_{0.0};
  double adc_back_edge_to_center_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_s_{0.0};
  double adc_heading_{0.0};
  bool is_boundary_{false};
  Polygon2d adc_polygon_{};
  Polygon2d back_boundary_polygon_{};
  Boundary back_boudnary_{};
  double merge_heading_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  std::unordered_map<int, AttentionObsInfo> obs_on_merging_lane_{};

  // int run_time_in_cycle_{-1};

 private:
  const double kApproximateEqualZero{1e-4};
  const config::AutoPlanningResearchConfig::SpeedVehicleBackCipvDeciderConfig*
      motorway_speed_back_cipv_decider_config_ptr_{nullptr};
  bool execute_decelerate_for_merge_ = false;
};
REGISTER_SCENARIO_TASK(MotorwaySpeedConfilictMergeInDecider);
}  // namespace planning
}  // namespace neodrive
