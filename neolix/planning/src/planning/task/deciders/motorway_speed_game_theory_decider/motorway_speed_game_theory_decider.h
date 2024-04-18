#pragma once
#include <vector>

#include "src/planning/common/obstacle/decision_data.h"
#include "src/planning/math/level_k/level_k.h"
#include "src/planning/math/level_k/mcts.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {
class MotorwaySpeedGameTheoryDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedGameTheoryDecider);

 public:
  using Vehicle = level_k_context::Vehicle;
  using Actions = level_k_context::Actions;
  using Weights = level_k_context::Weights;
  using Believes = level_k_context::Believes;
  using Node = level_k_context::Node;
  using Trajectory = level_k_context::Trajectory;
  using State = level_k_context::State;
  using Param = level_k_context::Param;
  using OrinTrajectory = level_k_context::OrinTrajectory;
  using TrajPoint = level_k_context::TrajPoint;

  virtual ~MotorwaySpeedGameTheoryDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;

  void SaveTaskResults(TaskInfo& task_info) override{};

  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  bool FilterAgents(
      TaskInfo& task_info,
      std::vector<Obstacle*>& filter_agents);  // Full plane obstacle filtering

  bool CalcVehInfoFromPrediction(TaskInfo& task_info);

  void CalcAgentGoal(Vehicle& agent);

  bool CalcLevelK(TaskInfo& task_info);  // execute level k for single agent

  bool CalOrinTrajectory(const Obstacle& obs, Vehicle& agent);

  bool SaveSnapshot(TaskInfo& task_info);

  bool GetSnapshot(TaskInfo& task_info);

 private:
  mcts_u::Node node_;
  mcts_u::State state_;
  Vehicle ego_;
  std::vector<Vehicle> agents_;
  std::vector<Vehicle> last_agents_;
  std::vector<std::vector<Vehicle>> agents_set;
  double time_step_{0.1};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedGameTheoryDecider);

}  // namespace planning
}  // namespace neodrive