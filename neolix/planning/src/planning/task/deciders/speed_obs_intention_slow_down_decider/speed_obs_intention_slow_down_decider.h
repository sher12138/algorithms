#pragma once

#include <cmath>

#include "obstacles_intention.pb.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/task/deciders/motorway_speed_conflict_decider/conflict_data_base_map.h"
#include "src/planning/util/speed_planner_common.h"

namespace neodrive {
namespace planning {

class SpeedObsIntentionSlowDownDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedObsIntentionSlowDownDecider);

 public:
  virtual ~SpeedObsIntentionSlowDownDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool ObsIntentionProcess(TaskInfo& task_info);
  bool SelectIntentionObs(
      std::unordered_map<int, double>& turn_right_obs_ids_probs,
      std::unordered_set<int>& agent_ids,
      std::unordered_set<int>& cutin_agent_ids);
  bool GetRightConnectionConflictInfos(
      TaskInfo& task_info, const std::unordered_set<int>& agent_ids);
  bool ProcessRightConnectionConflictInfos(
      std::unordered_set<int>& agent_ids,
      std::unordered_map<int, double>& turn_right_obs_ids_probs);
  bool ProcessNoneRightConflictInfos(
      TaskInfo& task_info, const std::unordered_set<int>& agent_ids,
      std::unordered_map<int, double>& turn_right_obs_ids_probs);
  bool ProcessNormalTurnRightObs(
      TaskInfo& task_info, std::unordered_set<int>& agent_ids,
      std::unordered_map<int, double>& turn_right_obs_ids_probs);
  bool ProcessOneNormalTurnRightObs(
      TaskInfo& task_info, int& agent_id,
      std::unordered_map<int, double>& turn_right_obs_ids_probs);
  bool ProcessRectifiedTurnRightObs(
      std::unordered_set<int>& agent_ids,
      std::unordered_map<int, double>& turn_right_obs_ids_probs);
  bool ProcessCutinObs(TaskInfo& task_info,
                       std::unordered_set<int>& cutin_agent_ids);
  bool ProcessOneCutinObs(TaskInfo& task_info, int& agent_id);

 private:
  std::unordered_map<
      int, std::vector<neodrive::global::planning::ObstaclesIntention>>
      cur_frame_obs_intention_;
  std::unordered_map<int, ObstacleIntentionContext> cur_obs_intention_context_;

  bool intention_slow_down_{false};
  double cur_speed_{0.0};
  ConflictDataBaseMap conflict_data_base_map_;
  std::vector<ConnectionConflictInfo> cur_right_connection_conflict_infos_;
  static constexpr double kJunctionRadius = 13.0;
  static constexpr double kTTSGo = 1.52;
  static constexpr double kEgoSlowTimeBuffer = 8.0;
  static constexpr double kEgoStrongSlowTimeBuffer = 2.5;
  static constexpr double kEgoLengthBuffer = 1.6;
  static constexpr double kEgoCarStopVelThreshold = 0.05;
  static constexpr double kMaxDistance = 1000.0;
  static constexpr double kMaxSpeed = 100.0;
  static constexpr double kACCSlowLimit = -1.5;
  static constexpr double kACCSlowWeight = 1.5;
  static constexpr double kEnableACCSlowThreshold = 1.6;
  static constexpr double kProbThreshold = 0.5;
  static constexpr double kTimeToAreaBuffer = 0.1;
  static constexpr double kEgoDisStartBuffer = 12.0;
  static constexpr double kEgoDisSlowBuffer = 15.0;
  static constexpr double kSqrtSeachDisBuffer = 36.0;
  static constexpr double kHighSpeedCutinSlowHighRatio = 0.75;
  static constexpr double kHighSpeedCutinSlowMediRatio = 0.5;
  static constexpr double kCutinTimeBuffer = 0.5;
  static constexpr double kCutinSpeedBuffer = 0.5;
  static constexpr double kCutinTimeSlowBuffer = 2.0;
  static constexpr double kMinNum = 0.001;
  double speed_limit_ = kMaxSpeed;
  bool is_strong_slow_ = false;
  PathData* last_path_data_ = nullptr;
  PathPoint* ego_path_point_ = nullptr;
  double ego_current_s_in_trajectort_ = 0.0;
  bool is_in_junction_ = false;
};

REGISTER_SCENARIO_TASK(SpeedObsIntentionSlowDownDecider);

}  // namespace planning
}  // namespace neodrive
