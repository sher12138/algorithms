#pragma once

#include <queue>
#include "algo_and_model.h"
#include "common/math/util.h"
#include "obs_sequence_data.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

class MotorwaySpeedAsternProtcDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedAsternProtcDecider);

 public:
  virtual ~MotorwaySpeedAsternProtcDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void PrintAllObsInfo();
  bool UpdateSequenceData(TaskInfo& task_info);
  bool ObstacleValidCheck(const Obstacle& obs);
  void Reset() override;

 private:
  bool Init(TaskInfo& task_info);
  void InitEgoBaseInfo(TaskInfo& task_info);
  void ExtractNeedHandleObsBaseHistoryInfo(TaskInfo& task_info);
  void StaticProtc(TaskInfo& task_info);
  bool CalSafeStopTime(TaskInfo& task_info);

  // void RealDistanceToObs(TaskInfo& task_info, Obstacle& obs);
  bool Process(TaskInfo& task_info);
  bool UpdateProtectedLimitSpeed(const double lateral_distance,
                                 const double diff_lateral);

 private:
  // new seq:
  // Base info
  double safe_stop_time_{};
  Boundary max_check_boundary_{};
  Boundary min_safe_boundary_{};
  std::unordered_set<int> need_handle_obs_id_vec_{};
  std::unordered_map<int, double> obs_id_to_heading_diff_map_{};
  std::unordered_map<int, double> obs_id_to_path_heading_map_{};

  EgoState ego_state_{};
  TimeSequenceData& time_sequence_data_ = TimeSequenceData::Instance();

  std::vector<int> has_covered_ele_{};

  // ans info:
  bool whether_turn_scenario_{false};
  double speed_limit_{};
  double deaccelerate_{};
  bool update_speed_limit_{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedAsternProtcDecider);

}  // namespace planning
}  // namespace neodrive
