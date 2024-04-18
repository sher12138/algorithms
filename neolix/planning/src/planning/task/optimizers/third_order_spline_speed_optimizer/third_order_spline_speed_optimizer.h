#pragma once

#include <string>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "third_order_spline_speed_model.h"
#include "third_order_spline_speed_mpc.h"

namespace neodrive {
namespace planning {

class ThirdOrderSplineSpeedOptimizer final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(ThirdOrderSplineSpeedOptimizer);

 public:
  virtual ~ThirdOrderSplineSpeedOptimizer() override;

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
  std::shared_ptr<ThirdOrderSplineSpeedMPC> third_order_spline_mpc_{nullptr};

  double delta_t_{0.4};
  double dense_time_{0.05};
  std::size_t n_{0};
  ThirdOrderSplineSpeed::State init_state_{};      //{0.0, v0, a0}
  ThirdOrderSplineSpeed::Control init_control_{};  //{j0}
  std::vector<double> upper_s_bounds_{};           // dp_st_data.upper_tunnel
  std::vector<double> lower_s_bounds_{};           // dp_st_data.lower_tunnel
  std::vector<double> upper_v_bounds_{};
  std::vector<double> lower_v_bounds_{};  // from v0 and speed limit
  std::vector<double> upper_a_bounds_{};
  std::vector<double> lower_a_bounds_{};  // from config:-4.0~4.0
  std::vector<double> upper_jerk_bounds_{};
  std::vector<double> lower_jerk_bounds_{};  // from config:-3.0~3.0

  std::vector<STGoalSInfo> goal_upper_s_{};  // dp_st_data.goal_s_upper
  std::vector<STGoalVInfo> goal_upper_v_{};  // dp_st_data.goal_v_upper
  std::vector<STGoalSInfo> goal_lower_s_{};  // dp_st_data.goal_s_lower
  std::vector<STGoalVInfo> goal_lower_v_{};
  std::vector<STGoalAInfo> goal_a_{};  // dp_st_data.goal_v_lower

  std::vector<ThirdOrderSplineSpeed::OptVariables> opt_variables_{};
};
REGISTER_SCENARIO_TASK(ThirdOrderSplineSpeedOptimizer);
}  // namespace planning
}  // namespace neodrive