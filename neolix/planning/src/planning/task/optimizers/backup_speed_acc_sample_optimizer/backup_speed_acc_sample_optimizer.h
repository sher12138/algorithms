#pragma once

#include <cmath>
#include <vector>

#include "reference_line/reference_line_util.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class BackupSpeedAccSampleOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(BackupSpeedAccSampleOptimizer);

 public:
  virtual ~BackupSpeedAccSampleOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};

  void Reset() override{};
  void Clear();

  bool CalMaxAccByMainPlanner(
      const BackupCipv& backup_cipv,
      const SpeedObstacleContext& speed_obstacle_context, double& max_a);

  void UpdateMaxA(const SpeedObstacleDecision& obs_dec, double& max_a);

  bool ResampleAcc(TaskInfo& task_info, double acc);
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  void StoreSpeedResult(TaskInfo& task_info);

  void CallSlowDown(TaskInfo& task_info);

 private:
  std::vector<double> acc_sample_{};
  std::vector<SpeedPoint> speed_point_{};
  double resolution_{0.05};
  double time_length_{8.0};
  double adc_current_v_{0.0};
  double need_min_acc_{0.0};

  double sentinal_a_{0.0};
  bool pass_check_{false};

  bool is_in_door_{false};
};

REGISTER_SCENARIO_TASK(BackupSpeedAccSampleOptimizer);

}  // namespace planning
}  // namespace neodrive
