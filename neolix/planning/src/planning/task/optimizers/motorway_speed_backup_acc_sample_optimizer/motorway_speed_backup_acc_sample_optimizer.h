#pragma once

#include <cmath>
#include <vector>

#include "reference_line/reference_line_util.h"
#include "src/planning/public/planning_lib_header.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedBackupAccSampleOptimizer final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedBackupAccSampleOptimizer);

 public:
  virtual ~MotorwaySpeedBackupAccSampleOptimizer() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};

  void Reset() override{};
  void Clear();

  bool CalMaxAccByMainPlanner(
      const BackupCipv& backup_cipv,
      const MotorwaySpeedObstacleContext& speed_obstacle_context,
      double& max_a);

  void UpdateMaxA(const MotorwayMultiCipvSpeedObstacleDecision& obs_dec,
                  double& max_a);
  void GenerateFinalTrajectory(TaskInfo& task_info);
  bool ResampleAcc(TaskInfo& task_info,
                   double acc);  // range sample, bi_sec
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  void StoreSpeedResult(TaskInfo& task_info);

  void CallSlowDown(TaskInfo& task_info);

 private:
  double need_min_acc_{0.0};
  std::vector<double> acc_sample_{};
  std::vector<SpeedPoint> speed_point_{};
  double resolution_{0.05};
  double time_length_{8.0};
  SLPoint adc_current_sl_{};
  double adc_current_v_{0.0};
  Vec2d adc_current_pos_{};
  double adc_heading_{0.0};

  std::vector<std::pair<double, double>> sentinal_a_{};
  std::vector<double> history_ans_{};

  bool is_in_door_{false};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedBackupAccSampleOptimizer);

}  // namespace planning
}  // namespace neodrive
