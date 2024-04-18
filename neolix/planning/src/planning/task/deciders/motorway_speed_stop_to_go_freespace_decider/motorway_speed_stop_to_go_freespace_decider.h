#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

class MotorwaySpeedStopToGoFreespaceDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedStopToGoFreespaceDecider);

 public:
  virtual ~MotorwaySpeedStopToGoFreespaceDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);
  bool DataCheck(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  void GetAdcCheckPolygons(TaskInfo& task_info);
  void UpdatedLimitedSpeed();
  bool SegmentCutByPolygon(const Polygon2d& polygon, const Segment2d& segment);
  void ProcessFreespaceCollisionRisk(TaskInfo& task_info,
                                     const Polygon2d& freespace_polygon);

 private:
  std::vector<Polygon2d> adc_check_polygons_{};

 private:
  bool update_limited_speed_{false};
  double target_speed_{1e5};
  double limited_speed_{std::numeric_limits<double>::infinity()};
  double limited_deceleration_{0.0};
  double last_limited_speed_{std::numeric_limits<double>::infinity()};

 private:
  double adc_current_l_{0.0};
  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_current_v_{0.0};
  int obs_keep_cnt_{0};
  int request_sd_cnt_{0};

 private:
  static constexpr double kPlanningCycleTime{0.1};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedStopToGoFreespaceDecider);
}  // namespace planning
}  // namespace neodrive
