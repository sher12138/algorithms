#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class BiasDrivingDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(BiasDrivingDecider);

 public:
  virtual ~BiasDrivingDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool ProcessStationFunction(TaskInfo& task_info, const double& min_left_bound,
                              const double& min_right_bound, bool& skip_flag);

  bool ProcessTriggeredStopFunction(TaskInfo& task_info,
                                    const double& min_left_bound,
                                    const double& min_right_bound,
                                    bool& skip_flag);

  bool ProcessBroadRoadFunction(TaskInfo& task_info,
                                const double& min_left_bound,
                                const double& min_right_bound, bool& skip_flag);

  bool CalcFrontLaneBound(const TaskInfo& task_info, double& min_left_bound,
                          double& min_right_bound);

  bool ProcessRoadHasBound(TaskInfo& task_info, const double min_left_bound,
                           const double min_right_bound, bool& finish_flag);

  bool IsInRoadHasBound(const TaskInfo& task_info, bool& flag);

  bool ProcessRoadHasBoundAdjacentJunction(const TaskInfo& task_info,
                                           double& keep_bias_distance);

  void YieldDynamicObsSideBack(TaskInfo& task_info, const double& left_bound,
                               const double& right_bound);
  bool IsFrontLaneTurning(const TaskInfo& task_info,
                          bool& is_front_lane_turning);

  bool IsJunctionClose(const TaskInfo& task_info, bool& is_junction_close);
  void ComputePullOverRefL(TaskInfo& task_info, const double min_left_bound,
                           const double min_right_bound,
                           double bias_distance_to_bound, bool bias_to_right);
  bool CreateYieldVirtualObs(TaskInfo& task_info);

 private:
  bool is_front_lane_turning_{false};
  bool is_junction_close_{false};
  bool is_need_bias_driving_{false};
  // double bias_driving_value_{0.};
  double keep_bias_distance_{0.};
  double bias_distance_{0.};
  double filter_k_{0.};
};

REGISTER_SCENARIO_TASK(BiasDrivingDecider);

}  // namespace planning
}  // namespace neodrive
