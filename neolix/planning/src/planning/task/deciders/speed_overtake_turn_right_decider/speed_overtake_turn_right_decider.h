#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

struct ObsPredictionInfo {
  // GoDirectionType obs_go_direction_{GoDirectionType::GO_STRAIGHT};
  bool obs_will_turn{false};
  double accum_heading_diff{0.0};
  bool updated{false};

  void Reset() {
    // obs_go_direction_ = GoDirectionType::GO_STRAIGHT;
    obs_will_turn = false;
    accum_heading_diff = 0.0;
    updated = false;
  }
};

class SpeedOvertakeTurnRightDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedOvertakeTurnRightDecider);

 public:
  virtual ~SpeedOvertakeTurnRightDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  void DealTurnRightCollisionRisk(TaskInfo& task_info, const Obstacle* obs);
  void LimitSpeedAtRiskArea(TaskInfo& task_info, const Obstacle* obs);
  void LimitSpeedByObs(TaskInfo& task_info, const Obstacle* obs);
  void UpdatedLimitedSpeed();
  void GetNewObsDecision(TaskInfo& task_info, const Obstacle* obs);
  void ModifyOriginObsDecision(TaskInfo& task_info);
  bool CollisionCheckWithPredictionTrajectory(TaskInfo& task_info,
                                              const Obstacle& obstacle,
                                              SpeedObstacleDecision& decision);
  bool Init(TaskInfo& task_info);
  bool InitCollisionCheckArea(const ReferenceLinePtr& ref_ptr,
                              const InsidePlannerData& inside_data,
                              const Boundary& adc_boundary);
  bool DataCheck(TaskInfo& task_info);
  void UpdateObsDecisionInfo(TaskInfo& task_info);
  void ClearObsHistoryInfo();
  void UpdataObsHeadingInfo(TaskInfo& task_info, const Obstacle* const obs);
  void UpdateObsPredictionInfo(TaskInfo& task_info, const Obstacle* const obs);
  void UpdateDynamicToStaticObs(TaskInfo& task_info);
  void SearchRiskArea(TaskInfo& task_info);
  bool IsInRightTurnRiskArea(TaskInfo& task_info);
  void FindHighAndLowWithPolygon(const std::vector<Box2d>& adc_bounding_boxes,
                                 const Polygon2d& obstacle_box, bool* find_high,
                                 bool* find_low, std::size_t* high_index,
                                 std::size_t* low_index);

 private:
  Boundary check_obs_turn_right_area_{};
  std::unordered_map<int, ObsHeadingInfo> obs_heading_info_{};
  std::unordered_map<int, ObsPredictionInfo> obs_prediction_info_{};
  // obs_decision_info_ : obs id, <if decision result updated, decision result>
  std::unordered_map<int, std::pair<bool, SpeedObstacleDecision>>
      obs_decision_info_{};
  GoDirectionType car_go_direction_{GoDirectionType::GO_STRAIGHT};
  Polygon2d adc_polygon_{};
  MapTurnRightRiskAreaInfo risk_area_info_{};

 private:
  double adc_current_s_{0.0};
  double adc_back_edge_s_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_v_{0.0};
  double target_speed_{1e5};
  bool update_limited_speed_{false};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  bool in_right_turn_risk_area_{false};
  double strategy_max_limit_speed_{0.0};

 private:
  static constexpr double kPlanningCycleTime{0.1};
  static constexpr double kZero{1e-4};
};

REGISTER_SCENARIO_TASK(SpeedOvertakeTurnRightDecider);
}  // namespace planning
}  // namespace neodrive
