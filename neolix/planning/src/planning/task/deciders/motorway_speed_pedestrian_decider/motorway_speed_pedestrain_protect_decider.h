#pragma once

#include <deque>

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

struct DynPedestrianInfo {
  std::deque<Boundary> boundary_history{};
  std::deque<double> theta{};
  std::deque<double> velocity{};
  int lost_cnt{0};
  void Reset() {
    boundary_history.clear();
    theta.clear();
    velocity.clear();
    lost_cnt = 0;
  }
};
struct PedestrianAttentionBound {
  double s;
  double l;
  double l_s;
  double l_e;
};

class MotorwaySpeedPedestrianProtectDecider final
    : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedPedestrianProtectDecider);

 public:
  virtual ~MotorwaySpeedPedestrianProtectDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Process(TaskInfo& task_info);
  bool Init(TaskInfo& task_info);

  bool DataCheck(TaskInfo& task_info);
  void ClearObsHistoryInfo();
  bool ComputeFrontCautionBoundAlongPath(const InsidePlannerData& inside_data,
                                         const PathData& path_data);
  void ExtractPedestrianInfo(TaskInfo& task_info);
  void UpdataDynPedestrianInfo(const Obstacle* const obs);

  double ComputeStaticPedestrianSpeedLimit();
  double ComputeDynamicPedestrianSpeedLimit();

  bool HasOverlapWithAttentionBound(
      const std::vector<PedestrianAttentionBound>& attention_bound,
      const Boundary& boundary, int* index) const;
  std::array<double, 3> ComputeStaticEndState(
      const std::pair<int, int>& static_info);
  std::array<double, 3> ComputeDynamicEndState(
      const std::pair<int, int>& dynamic_info);

 private:
  std::unordered_map<int, DynPedestrianInfo> pedestrian_info_{};
  std::vector<PedestrianAttentionBound> static_attention_bound_{};
  std::vector<PedestrianAttentionBound> dynamic_attention_bound_{};
  std::unordered_set<int> ignore_pedestrain_{};

 private:
  double adc_current_v_{0.0};
  double adc_current_l_{0.0};
  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
  Boundary adc_boundary_{};

  double static_lateral_buffer_{0.0};
  double dynamic_lateral_buffer_{0.0};
  double max_speed_{0.};
  std::array<double, 3> init_state_{};

  double static_pedestrian_limit_{0.};
  double dynamic_pedestrian_limit_{0.};
  bool update_limited_speed_{false};

 private:
  static constexpr double kPlanningCycleTime{0.1};
  static constexpr double kZero{1e-4};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedPedestrianProtectDecider);

}  // namespace planning
}  // namespace neodrive
