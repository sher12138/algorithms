#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

struct ObsParalleInfo {
  std::deque<double> lat_dis{};
  std::deque<double> lon_dis{};
  std::deque<double> obs_speed{};
  int lost_cnt{0};
  int parallel_cnt{0};
  int lat_close_cnt{0};

  void Reset() {
    lat_dis.clear();
    lon_dis.clear();
    obs_speed.clear();
    lost_cnt = 0;
    parallel_cnt = 0;
    lat_close_cnt = 0;
  }
};

class MotorwaySpeedYieldParalleledObsDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedYieldParalleledObsDecider);

 public:
  virtual ~MotorwaySpeedYieldParalleledObsDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  bool Init(TaskInfo& task_info);
  bool InitCollisionCheckArea(const Boundary& adc_boundary);
  void ClearObsHistoryInfo();
  void GetCollideDecisionObs(TaskInfo& task_info);
  bool DataCheck(TaskInfo& task_info);
  ErrorCode DynamicObstacleParallelCheck(TaskInfo& task_info);
  void UpdataParalleledObsInfo(TaskInfo& task_info, const Obstacle* const obs);
  void GetObsParallelResult(const Obstacle* const obs, bool& is_parallel_obs,
                            double obs_correct_min_parallel_dis);
  void DealSideRiskObs(const Obstacle* const obs);
  void UpdatedLimitedSpeed();

 private:
  Boundary adc_left_collision_check_area_{};
  Boundary adc_right_collision_check_area_{};
  std::unordered_map<int, CollisionInfo> has_collision_dynamic_obs_{};
  std::unordered_map<int, ObsParalleInfo> paralleled_obs_info_{};
  GoDirectionType car_go_direction_{GoDirectionType::GO_STRAIGHT};
  Polygon2d adc_polygon_{};

 private:
  double adc_current_l_{0.0};
  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_back_edge_s_{0.0};
  double adc_current_v_{0.0};
  double min_parallel_dis_{0.5};

 private:
  bool update_limited_speed_{false};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double target_speed_{1e5};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};

 private:
  static constexpr double kPlanningCycleTime{0.1};
  static constexpr double kZero{1e-4};
  static constexpr double kSideAreaWidth{2.0};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedYieldParalleledObsDecider);
}  // namespace planning
}  // namespace neodrive
