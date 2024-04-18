#pragma once

#include "common/math/util.h"
#include "planning/scenario_manager/scenario_common.h"
#include "src/planning/common/data_center/em_planning_data.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {
namespace speed_planner_common {

bool Reverse(const Obstacle* obstacle, const PathData& path_data,
             double heading);

void MaxDecelCalculator(const InsidePlannerData& inside_data,
                        const OutsidePlannerData& outside_data);

bool RoadBoundSafeCheck(const ReferenceLinePtr& reference_line,
                        const InsidePlannerData& inside_data,
                        OutsidePlannerData& outside_planner_data,
                        PathData* const path_data, bool& fail_safe_stop);

bool PerceptionCurbSafeCheck(const InsidePlannerData& inside_data,
                             PathData* const path_data);

bool SpeedDataSanityCheck(const std::vector<SpeedPoint>* speed_pts);

bool LmsSensorCheck(const InsidePlannerData& inside_data,
                    PathData* const path_data, bool& fail_safe_stop);

bool IsStaticObsNeedIgnore(const SLPoint& vel_sl, const Boundary& obs_boundary,
                           const double& left_road_bound,
                           const double& right_road_bound, bool& is_valid,
                           const bool is_forward = true);

bool IsDynamicObsNeedIgnore(const InsidePlannerData& inside_data,
                            const Obstacle& obstacle, bool& is_valid);

PublishableTrajectory CombineSpeedAndPath(
    TrajectoryPoint init_point, const PathData& path_data,
    const std::vector<SpeedPoint>& speed_pts, const double station_error,
    const bool relative_time = true);

void ExtractValidObstacle(const MotorwaySpeedObstacleContext& speed_obs_context,
                          std::vector<Obstacle>& valid_obs);

void ExtractValidObstacle(const SpeedObstacleContext& speed_obs_context,
                          std::vector<Obstacle>& valid_obs);

void ExtractReverseObstacle(
    const std::vector<MotorwayMultiCipvSpeedObstacleDecision>&
        has_collsion_obstacle,
    const PathData& path_data, double heading,
    std::unordered_set<int>& reverse_obs_id);

void ExtractReverseObstacle(
    const std::vector<SpeedObstacleDecision>& has_collsion_obstacle,
    const PathData& path_data, double heading,
    std::unordered_set<int>& reverse_obs_id);

bool GetAdcEnlargeBuffer(const InsidePlannerData& inside_data,
                         const std::size_t curr_mode, double* en_large_buffer,
                         double* front_en_large_buffer,
                         double* back_en_large_buffer);

bool FinalObstacleCollisionSanityCheck(
    const std::vector<Obstacle>& need_check_obs_list,
    const std::unordered_set<int>& reverse_obs_id,
    const PublishableTrajectory& combined_trajectory, double& collision_time,
    double query_t = 2.0);

bool FinalObstacleCollisionSanityCheck(
    TrajectoryPoint init_point, const PathData& path_data,
    const MotorwaySpeedObstacleContext& obs_decision,
    const std::unordered_set<int>& need_ignore_obs_dy,
    const std::unordered_set<int>& need_ingore_obs_st,
    const double station_error, std::vector<SpeedPoint>& speed_point);

bool FinalObstacleCollisionSanityCheck(
    TrajectoryPoint init_point, const PathData& path_data,
    const SpeedObstacleContext& obs_decision,
    const std::unordered_set<int>& need_ignore_obs_dy,
    const std::unordered_set<int>& need_ignore_obs_st,
    const double station_error, std::vector<SpeedPoint>& speed_point);

std::string GetCurrentModeName(const std::size_t current_mode);

bool CreepProcess(const InsidePlannerData& inside_planner_data,
                  EMPlanningData* planning_data);

double CalSafeSpeedDeltaParallelAdc(const double heading_diff);

bool IsApproachingAdc(const Obstacle* const obs,
                      const InsidePlannerData& inside_data);

double GetObs2AdcLateralDis(const Obstacle& obs, double adc_current_l);
bool InMergingArea(TaskInfo& task_info);
bool InDivergingArea(TaskInfo& task_info);
bool JudgeDirectionCarWillGo(const ReferenceLinePtr& ref_ptr,
                             const InsidePlannerData& inside_data,
                             GoDirectionType& car_go_direction);

bool ObsEgoAlongPathHeadingDiff(const Obstacle* obstacle,
                                const PathData& path_data, double ego_heading,
                                double& heading_diff, double& path_heaeding);

bool ObsEgoAlongPathHeadingDiffBaseSpeedHeading(const Obstacle* obstacle,
                                                const PathData& path_data,
                                                double ego_heading,
                                                double& heading_diff);

bool ObsEgoAlongPathLateralMinDistance(const Obstacle* obstacle,
                                       const PathData& path_data,
                                       double& lateral_min_diff);
bool GetObsToPathLatDis(const std::vector<Boundary>& adc_sl_boundaries,
                        const Obstacle& obs, double& obs_path_lat_dis);
std::vector<Vec2d> GetOdoLidarPoints(
    const neodrive::global::perception::Freespace& freespace);

bool UnprotectStright(TaskInfo& task_info, double detect_distance);
bool EgoInTNonParkCross(TaskInfo& task_info);
bool EgoInIntersection(TaskInfo& task_info);
bool EgoInTParkCross(TaskInfo& task_info);
};  // namespace speed_planner_common
}  // namespace planning
}  // namespace neodrive
