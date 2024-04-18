#pragma once

#include <string>

#include "motorway_speed_model_for_iter.h"
#include "motorway_speed_mpc_for_iter.h"
#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedOptimizerForIter final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedOptimizerForIter);

 public:
  virtual ~MotorwaySpeedOptimizerForIter() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  ErrorCode Optimize(const InsidePlannerData& inside_data,
                     DecisionData* const decision_data,
                     OutsidePlannerData* const outside_data);

 private:
  ErrorCode STGenerator(const InsidePlannerData& inside_data,
                        DecisionData* const decision_data,
                        OutsidePlannerData* const outside_data,
                        std::vector<SpeedPoint>* speed_points);

  bool CalInitStateAndControl(const InsidePlannerData& inside_data,
                              DecisionData* const decision_data,
                              OutsidePlannerData* const outside_data);

  bool CalBounds(const InsidePlannerData& inside_data,
                 DecisionData* const decision_data,
                 OutsidePlannerData* const outside_data);

  bool CalGoalSV(const InsidePlannerData& inside_data,
                 DecisionData* const decision_data,
                 OutsidePlannerData* const outside_data);

  void ClearData();

 private:
  double speed_limit_{0.};
  std::shared_ptr<MotorwaySpeedMPCForIter> motorway_mpc_{nullptr};

  double delta_t_{0.4};
  double dense_time_{0.05};
  std::size_t n_{0};
  MotorwaySpeedForIter::State init_state_{};      //{0.0, v0, a0}
  MotorwaySpeedForIter::Control init_control_{};  //{j0}
  std::vector<double> upper_s_bounds_{};          // upper_tunnel
  std::vector<double> lower_s_bounds_{};          // lower_tunnel
  std::vector<double> upper_v_bounds_{};
  std::vector<double> lower_v_bounds_{};  // from v0 and speed limit
  std::vector<double> upper_a_bounds_{};
  std::vector<double> lower_a_bounds_{};  // from config:-4.0~4.0
  std::vector<double> upper_jerk_bounds_{};
  std::vector<double> lower_jerk_bounds_{};  // from config:-3.0~3.0

  std::vector<STPoint> goal_s_{};  // goal_s
  std::vector<STPoint> goal_v_{};  // goal_v
  std::vector<STPoint> goal_a_{};  // goal_a

  std::vector<MotorwaySpeedForIter::OptVariables> opt_variables_{};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedOptimizerForIter);

}  // namespace planning
}  // namespace neodrive
