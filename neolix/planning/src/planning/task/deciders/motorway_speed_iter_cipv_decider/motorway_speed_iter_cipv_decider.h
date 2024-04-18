#pragma once

#include "motorway_speed_iter_forward_generate.h"
#include "planning/planning_map/planning_map.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class MotorwaySpeedIterCipvDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedIterCipvDecider);

 public:
  virtual ~MotorwaySpeedIterCipvDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  void SaveLOGResults();

 private:
  bool Init(TaskInfo& task_info);

  void CheckConcernedObsDecision(TaskInfo& task_info);

  std::pair<double, int> FindClosestTimeAndIndex(
      const std::vector<double>& list, double target);

  void UpdataEgoDeductionStartState(int update_index);

  void DeleteCollisonData(int cut_index);

  void ProcessFastestDeduction();

  void CalUpperDecisionLimit(TaskInfo& task_info);

  void AddIntervalVirtualObs(
      const std::vector<MotorwaySpeedObsExtend>& filtered_segmentation_decision,
      std::vector<MotorwaySpeedObsExtend>& complete_list);

  void AddDeductionData(const DeductionEgoStateSequence& deduction_data);

  void SupplementCuttedObs(
      std::vector<MotorwaySpeedObsExtend>& filtered_decison);

  void IntegrateObsDecision(
      std::vector<MotorwaySpeedObsExtend>& filtered_segmentation_decision,
      std::vector<MotorwaySpeedObsExtend>& complete_list);

  void IterObsToHybirdIdm(
      const std::pair<MotorwaySpeedObsExtend, int>& obs_extend,
      ObsBoundPolySeris obs_extend_polypoint);

  DeductionCollisionResult CollisionCheck();

  DeductionCollisionResult CollisionCheckForTake(
      const DeductionEgoStateSequence& deduction_data);

  void AbnormalDataEliminate();

  void PidDeductionForVirtual(const MotorwaySpeedObsExtend& obs_extend);

  DeductionEgoStateSequence PidDeductionForReal(const int& obs_index);

  void NormalIdmDeduction(const int& obs_index);

  void SetUpBackupPara(TaskInfo& task_info, const int& id, const bool& if_real);

  bool Process(TaskInfo& task_info);

  void IterDeduction(TaskInfo& task_info, const int& fist_obs_overlap_index);

  void SetupIntervalSpeedLimit(TaskInfo& task_info);

  void BidirectionalIter(std::pair<MotorwaySpeedObsExtend, int>& obs_extend);

  void GenerateSBoundForObs(TaskInfo& task_info);

  void GenerateVBoundForEgo(TaskInfo& task_info);

  void GenerateFollowTakeDecision(TaskInfo& task_info);

 private:
  IterDeductionEgoState ego_current_state_{0., 0., 0., 0.};
  DeductionEgoStateSequence ego_state_sequence_{};
  std::vector<MotorwaySpeedObsExtend> completion_obs_list_{};
  std::vector<MotorwaySpeedObsExtend> original_obs_list_{};
  std::vector<MotorwayMultiCipvSpeedObstacleDecision> obs_list_{};
  DeductionEgoStateSequence ego_deduction_;
  std::vector<double> s_upper_bound_base_deduction_{};
  std::vector<double> s_lower_bound_base_deduction_{};
  std::vector<double> v_upper_bound_base_decision_s_{};
  double adc_current_s_{0.0};
  double min_v_bound_{8.33};
  double v_target_{8.33};
  double limit_a_{0.0};
  double virtual_obs_dis_{100.0};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedIterCipvDecider);

}  // namespace planning
}  // namespace neodrive
