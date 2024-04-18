#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

struct TraficLightInfo {
  double lower_s{1.e6};
  double lower_t{1.e6};

  void Reset() {
    lower_s = 1.e6;
    lower_t = 1.e6;
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

class SpeedStartSideBySideDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedStartSideBySideDecider);

 public:
  virtual ~SpeedStartSideBySideDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  void ProcessStopToGORisk();
  void PrepareData(TaskInfo& task_info);
  void PrepareTrafficLightData(
      const std::vector<SpeedObstacleDecision>& virtual_obstacle_decision);
  void PrepareRiskObsData(TaskInfo& task_info);
  bool DataCheck(TaskInfo& task_info);
  void ClearHistoryCnt();
  bool Init(TaskInfo& task_info);
  bool IsInIntersection(TaskInfo& task_info);
  bool InitSideBySideCheckArea(const ReferenceLinePtr& ref_ptr,
                               const InsidePlannerData& inside_data,
                               const Boundary& adc_boundary);

 private:
  Polygon2d adc_polygon_{};
  Boundary check_side_by_side_area_{};
  std::unordered_map<int, TraficLightInfo> last_red_light_{};
  std::unordered_map<int, TraficLightInfo> current_red_light_{};
  std::unordered_map<int, RiskObsInfo> stop_to_go_info_{};

 private:
  double adc_current_s_{0.0};
  double adc_back_edge_s_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_v_{0.0};
  bool update_limited_speed_{false};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};
  double risk_obs_judge_size_{0.0};
  bool in_intersection_{false};
  bool is_red_to_green_{false};
  bool stop_to_go_flag_{false};
};

REGISTER_SCENARIO_TASK(SpeedStartSideBySideDecider);
}  // namespace planning
}  // namespace neodrive
