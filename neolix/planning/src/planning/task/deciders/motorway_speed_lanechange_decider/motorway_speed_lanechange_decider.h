#pragma once

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/math/polygon2d.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
using MotorwayLaneChangeStageState =
    neodrive::global::planning::MotorwayLaneChangeStageState;

class MotorwaySpeedLaneChangeDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedLaneChangeDecider);

 public:
  virtual ~MotorwaySpeedLaneChangeDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  bool ObsNinePalaceGridDivision(TaskInfo& task_info);
  void ObstacleClassification(TaskInfo& task_info,
                              const LaneChangeCheckAreaBoundary& check_area);
  bool MssModelJudgeSafty(TaskInfo& task_info);
  bool ProcessLaneChangeSpeedAdjust(TaskInfo& task_info);
  void CalTcPointForLeftLaneChange();
  void SetMssMapLeftLaneChange(TaskInfo& task_info);

  void CalTcPointForRightLaneChange();
  void SetMssMapRightLaneChange(TaskInfo& task_info);

  void AdjustForLeftChange(TaskInfo& task_info);

  void AdjustForRightChange(TaskInfo& task_info);

  void LongtitudeMethodForLr();
  void LongtitudeMethodForLl();

  void LongtitudeMethodForRf();
  void LongtitudeMethodForRr();

  void VisLaneChangePara(TaskInfo& task_info);

  void GetLaneBounds(TaskInfo& task_info, double& left_lanechange_dis,
                     double& right_lanechage_dis,
                     LaneChangeCheckAreaBoundary& check_area);
  void SetVirtualObsForTerminal(TaskInfo& task_info);
  void GenerateCherkAreaBoundary(
      TaskInfo& task_info, const LaneChangeBoundPara& lane_change_bound_para,
      const LaneChangeBoundPara& back_lane_change_bound_para,
      LaneChangeCheckAreaBoundary& check_area, const double& expend_index);

 private:
  double adc_current_s_{0.0};
  double adc_current_l_{0.0};
  double adc_front_edge_s_{0.0};
  double adc_start_l_{0.0};
  double adc_end_l_{0.0};
  double adc_width_{0.0};
  double adc_length_{0.0};
  double adc_back_edge_to_center_{0.0};
  double adc_current_v_{0.0};
  double left_lc_lat_dis_{0.0};
  double right_lc_lat_dis_{0.0};
  double adj_acc_{0.0};
  // std::vector<Boundary> check_area_{};
  // std::vector<std::vector<Boundary>> check_areas_{};
  LaneChangeCheckAreaBoundary check_areas_{};
  std::pair<double, double> tc_para_{};
  LeftChangeTcInfo left_lc_tc_para_;
  RightChangeTcInfo right_lc_tc_para_;
  LeftChangeMssMapInitPoint left_change_mss_map_point_;
  RightChangeMssMapInitPoint right_change_mss_map_point_;
  // std::pair<double, double> coordinate_for_fr{};
  LeftChangeMssModelSaftyCheck left_lanechange_info_{};
  RightChangeMssModelSaftyCheck right_lanechange_info_{};
  LaneChangeAdjType adj_type_{LaneChangeAdjType::NONE};
  MotorwayLaneChangeStageState::State pre_stage_{
      MotorwayLaneChangeStageState::PREPARE};
  double lane_change_end_point_s_{0.0};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedLaneChangeDecider);

}  // namespace planning
}  // namespace neodrive
