#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_data_base_map.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_info_for_u_turn.h"

namespace neodrive {
namespace planning {
namespace {
struct ConflictObs {
  ConflictObs(int id, double dis, double obs_speed, double obs_heading_diff,
              double ego_dis, double obs_dis_to_road)
      : obs_id(id),
        obs_dis(dis),
        obs_v(obs_speed),
        obs_heading_diff_to_road(obs_heading_diff),
        ego_dis_to_area(ego_dis),
        obs_offset(obs_dis_to_road){};
  int obs_id{0};
  double obs_dis{100.0};
  double obs_v{0.0};
  double obs_heading_diff_to_road{0.0};
  double ego_dis_to_area{100.0};
  double obs_offset{0.0};
  ;
  void Reset() {
    obs_id = 0;
    obs_dis = 100.0;
    obs_v = 0.0;
    obs_heading_diff_to_road = 0.0;
    ego_dis_to_area = 100.0;
    obs_offset = 0.0;
  }
};
}  // namespace
class MotorwaySpeedUTurnCautionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedUTurnCautionDecider);

 public:
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  void ResetPara();
  void SaveLOGResults();

 private:
  bool Init(TaskInfo& task_info);
  bool Checkfeasibility(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  void CheckCautionObs(TaskInfo& task_info);
  void EstablishVirtualObs(TaskInfo& task_info);
  void VisUturnPara(TaskInfo& task_info);

 private:
  double adc_current_s_{0.0};
  double adc_current_l_{0.0};
  double adc_current_v_{0.0};
  bool have_meeting_{false};
  bool have_merging_{false};
  bool set_merge_area_obs_{false};
  double ego_to_virtual_dis_{1000.0};
  std::vector<ConflictObs> conflict_obs_{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedUTurnCautionDecider);

}  // namespace planning
}  // namespace neodrive
