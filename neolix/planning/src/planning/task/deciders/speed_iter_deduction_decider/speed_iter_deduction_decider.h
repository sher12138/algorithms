#pragma once

#include "planning/planning_map/planning_map.h"
#include "speed_iter_forward_generate.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
namespace neodrive {
namespace planning {

class SpeedIterDeductionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedIterDeductionDecider);

 public:
  virtual ~SpeedIterDeductionDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  void SaveLOGResults();

 private:
  bool Init(TaskInfo& task_info);
  void CheckConcernedObsDecision(TaskInfo& task_info);
  std::pair<double, int> FindClosestTimeAndIndex(
      const std::vector<double>& list, const double& target);
  void UpdataEgoDeductionStartState(const int& update_index);
  void DeleteCollisonData(const int& cut_index);
  void AddDeductionData(const DeductionEgoStateSequence& deduction_data);

  bool Process(TaskInfo& task_info);

  void SupplementCuttedObs(std::vector<SpeedObsExtend>& filtered_decison);

  void IntegrateObsDecision(
      std::vector<SpeedObsExtend>& filtered_segmentation_decision,
      std::vector<SpeedObsExtend>& complete_list);

  void SetupIntervalSpeedLimit(TaskInfo& task_info);

  void AbnormalDataEliminate();
  void AddIntervalVirtualObs(
      const std::vector<SpeedObsExtend>& filtered_segmentation_decision,
      std::vector<SpeedObsExtend>& complete_list);

  void CalUpperDecisionLimit(TaskInfo& task_info);

  void ProcessFastestDeduction();
  void IterObsToHybirdIdm(const std::pair<SpeedObsExtend, int>& obs_extend,
                          ObsBoundPolySeris obs_extend_polypoint);

  void PidDeductionForVirtual(const SpeedObsExtend& obs_extend);

  DeductionEgoStateSequence PidDeductionForReal(const int& obs_index);

  void NormalIdmDeduction(const int& obs_index);

  DeductionCollisionResult CollisionCheck();

  DeductionCollisionResult CollisionCheckForTake(
      const DeductionEgoStateSequence& deduction_data);

  void SetUpBackupPara(TaskInfo& task_info, const int& id, const bool& if_real);

  void GenerateSBoundForObs(TaskInfo& task_info);

  void BidirectionalIter(std::pair<SpeedObsExtend, int>& obs_extend);

  void IterDeduction(TaskInfo& task_info, const int& fist_obs_overlap_index);

  void GenerateVBoundForEgo(TaskInfo& task_info);
  bool JudgeIfRod(TaskInfo& task_info, double start_s, double obs_speed);

  void GenerateFollowTakeDecision(TaskInfo& task_info);

 private:
  IterDeductionEgoState ego_current_state_{0, 0, 0, 0};
  std::vector<SpeedObstacleDecision> all_decison_{};
  DeductionEgoStateSequence ego_state_sequence_{};
  std::vector<SpeedObsExtend> completion_obs_list_{};
  std::vector<SpeedObstacleDecision> obs_list_{};
  std::vector<SpeedObsExtend> original_obs_list_{};
  DeductionEgoStateSequence ego_deduction_;
  std::vector<double> s_upper_bound_base_deduction_{};
  std::vector<double> s_lower_bound_base_deduction_{};
  std::vector<double> v_upper_bound_base_decision_s_{};
  double adc_current_s_{0.0};
  double min_v_bound_{8.33};
  double v_target_{8.33};
  double limit_a_{0.0};
  BackupCipv true_cipv_{};
};
REGISTER_SCENARIO_TASK(SpeedIterDeductionDecider);
}  // namespace planning
}  // namespace neodrive