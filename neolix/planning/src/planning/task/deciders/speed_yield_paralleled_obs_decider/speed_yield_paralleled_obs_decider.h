#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

class SpeedYieldParalleledObsDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedYieldParalleledObsDecider);

 public:
  virtual ~SpeedYieldParalleledObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  bool Init(TaskInfo& task_info);
  bool InitCollisionCheckArea(const Boundary& adc_boundary);
  void GetCollideDecisionObs(TaskInfo& task_info);
  bool DataCheck(TaskInfo& task_info);
  ErrorCode DynamicObstacleParallelCheck(TaskInfo& task_info);
  bool AdcLeftParallelCheck(const InsidePlannerData& inside_data,
                            const Obstacle* const obs,
                            OutsidePlannerData* const outside_data);
  bool AdcRightParallelCheck(const InsidePlannerData& inside_data,
                             const Obstacle* const obs,
                             OutsidePlannerData* const outside_data);

 private:
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
  static constexpr double kSideAreaWidth{2.0};
};

REGISTER_SCENARIO_TASK(SpeedYieldParalleledObsDecider);
}  // namespace planning
}  // namespace neodrive
