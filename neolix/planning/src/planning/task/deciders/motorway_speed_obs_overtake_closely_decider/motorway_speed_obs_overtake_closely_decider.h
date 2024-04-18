#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

class MotorwaySpeedObsOvertakeCloselyDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedObsOvertakeCloselyDecider);

 public:
  virtual ~MotorwaySpeedObsOvertakeCloselyDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  ErrorCode DynamicObstacleCollisionRiskCheck(TaskInfo& task_info);
  bool Init(TaskInfo& task_info);
  bool InitCollisionCheckArea(const ReferenceLinePtr& ref_ptr,
                              const InsidePlannerData& inside_data,
                              const Boundary& adc_boundary);
  bool InitForntCollisionCheckAreaAlongPath(
      const InsidePlannerData& inside_data, const PathData& path_data);
  bool DataCheck(TaskInfo& task_info);
  void ClearObsHistoryInfo();
  void GetCollideDecisionObs(TaskInfo& task_info);
  bool AdcFrontCollisionCheck(const InsidePlannerData& inside_data,
                              const Obstacle* const obs,
                              const std::vector<PathPoint>& path_points,
                              OutsidePlannerData* const outside_data);
  bool AdcLeftCollisionCheck(const InsidePlannerData& inside_data,
                             const Obstacle* const obs,
                             OutsidePlannerData* const outside_data);
  bool AdcRightCollisionCheck(const InsidePlannerData& inside_data,
                              const Obstacle* const obs,
                              OutsidePlannerData* const outside_data);
  bool AdcBackCollisionCheck(const InsidePlannerData& inside_data,
                             const Obstacle* const obs,
                             OutsidePlannerData* const outside_data);
  void FindLowCollisionPointWithPolygon(const Polygon2d& obstacle_box,
                                        bool& find_low,
                                        std::size_t& low_index) const;
  void DealFrontObs(const Obstacle* const obs,
                    const std::vector<PathPoint>& path_points,
                    std::size_t low_index,
                    OutsidePlannerData* const outside_data);
  bool LeftSidePrepareData(
      const InsidePlannerData& inside_data,
      const Boundary& left_collision_check_area, const Obstacle* const obs,
      std::unordered_map<int, DynamicObsCollisionCheckData>&
          left_dynamic_obstacles_data);
  bool RightSidePrepareData(
      const InsidePlannerData& inside_data,
      const Boundary& right_collision_check_area, const Obstacle* const obs,
      std::unordered_map<int, DynamicObsCollisionCheckData>&
          right_dynamic_obstacles_data);
  void DealSideBigObs(const Obstacle* const obs,
                      std::unordered_map<int, DynamicObsCollisionCheckData>&
                          side_dynamic_obstacles_data);
  void DealSideSmallObs(const Obstacle* const obs,
                        std::unordered_map<int, DynamicObsCollisionCheckData>&
                            side_dynamic_obstacles_data);
  void ObsOvertakeAdc(const Obstacle* const obs,
                      DynamicObsCollisionCheckData& side_dynamic_obs_data);
  void AdcOvertakeObs(const Obstacle* const obs,
                      DynamicObsCollisionCheckData& side_dynamic_obs_data);
  bool IsEasyHardBrakeObs(const Obstacle* const obs);
  void UpdataObsLongInfo(TaskInfo& task_info, const Obstacle* const obs);
  void UpdataObsCutinInfo(TaskInfo& task_info, const Obstacle* const obs);
  void UpdataObsHeadingInfo(TaskInfo& task_info, const Obstacle* const obs);
  void CheckIfDynamicToStatic(TaskInfo& task_info);
  bool IsAdcNearJunction(TaskInfo& task_info);
  void GetObsLongResult(const Obstacle* const obs, bool& is_overtake_obs);
  void GetObsLatResult(const Obstacle* const obs, LatResult& lat_res);
  void GetObsTurningInfo(const Obstacle* const obs, bool& is_turning_obs);

 private:
  Boundary adc_back_collision_check_area_{};
  Boundary adc_left_collision_check_area_{};
  Boundary adc_right_collision_check_area_{};
  std::unordered_map<int, CollisionInfo> has_collision_dynamic_obs_{};
  std::unordered_map<int, ObsSInfo> obs_s_info_{};
  std::unordered_map<int, ObsCutinInfo> obs_cutin_info_{};
  std::unordered_map<int, ObsHeadingInfo> obs_heading_info_{};
  std::vector<Box2d> adc_boundaries_front_check_{};
  GoDirectionType car_go_direction_{GoDirectionType::GO_STRAIGHT};
  Polygon2d adc_polygon_{};

 private:
  double adc_current_l_{0.0};
  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
  bool update_limited_speed_{false};
  double adc_current_v_{0.0};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};

 private:
  static constexpr double kPlanningCycleTime{0.1};
  static constexpr double kZero{1e-4};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedObsOvertakeCloselyDecider);
}  // namespace planning
}  // namespace neodrive
