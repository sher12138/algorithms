#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

struct ObsHistoryInfo {
  Obstacle* obs_ptr{nullptr};
  bool is_static{false};
  int lost_cnt{0};
  // heading
  std::deque<double> heading_diff{};
  double accum_heading_diff{0.0};
  double last_heading{0.0};
  // lateral distance
  std::deque<double> lat_dis{};
  double last_lat_dis{0.0};

  void Reset() {
    obs_ptr = nullptr;
    is_static = false;
    lost_cnt = 0;
    // heading
    heading_diff.clear();
    last_heading = 0.0;
    accum_heading_diff = 0.0;
    // lateral distance
    lat_dis.clear();
    last_lat_dis = 0.0;
  }
};

struct RiskObsInfo {
  Obstacle* obs_ptr{nullptr};
  bool is_static{false};
  int cnt{0};

  void Reset() {
    obs_ptr = nullptr;
    is_static = false;
    cnt = 0;
  }
};

struct LeftTurnAreaInfo {
  double next_check_s = 0.0;
  double left_turn_start_s = 0.0;
  double left_turn_end_s = 0.0;

  void Reset() {
    next_check_s = 0.0;
    left_turn_start_s = 0.0;
    left_turn_end_s = 0.0;
  }
};

class SpeedStopToGiveWayDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedStopToGiveWayDecider);

 public:
  virtual ~SpeedStopToGiveWayDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  void ProcessStopToGORisk(TaskInfo& task_info);
  void RecognizeOvertakeTurnObs(TaskInfo& task_info);
  void RecognizeBlockAdcTurnObs(TaskInfo& task_info);
  bool HaveFrontAcrossRisk(const ObsHistoryInfo& obs_history_info);
  void ObserveLeftNearBigObs(TaskInfo& task_info);
  void UpdataDynamicObsInfo(Obstacle* obs);
  void UpdateDynamicToStaticObs(Obstacle* obs);
  bool Init(TaskInfo& task_info);
  void ClearObsHistoryInfo();
  void SearchRiskArea(TaskInfo& task_info);
  bool IsInIntersection(TaskInfo& task_info);
  void JudgeIfAdcWillTurnLeft(TaskInfo& task_info);
  void InitTurnLeftArea(TaskInfo& task_info);
  bool InitSideBySideCheckArea(const ReferenceLinePtr& ref_ptr,
                               const InsidePlannerData& inside_data,
                               const Boundary& adc_boundary);
  bool ObsInGiveWayArea(TaskInfo& task_info, const Obstacle* obs_ptr);

 private:
  Boundary adc_original_boundary_{};
  Polygon2d adc_polygon_{};
  Boundary check_left_overtake_turn_area_{};
  std::unordered_map<int, ObsHistoryInfo> obs_history_info_{};
  // record obstacles need to give way
  std::unordered_map<int, RiskObsInfo> risk_obs_info_{};
  LeftTurnAreaInfo left_turn_info_{};
  MapTurnRightRiskAreaInfo risk_area_info_{};

 private:
  double adc_current_s_{0.0};
  double adc_current_l_{0.0};
  double adc_back_edge_s_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_heading_{0.0};
  double adc_current_v_{0.0};
  bool update_limited_speed_{false};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  double risk_obs_min_size_{0.0};
  bool in_intersection_{false};
  bool adc_will_turn_left_{false};
};

REGISTER_SCENARIO_TASK(SpeedStopToGiveWayDecider);
}  // namespace planning
}  // namespace neodrive
