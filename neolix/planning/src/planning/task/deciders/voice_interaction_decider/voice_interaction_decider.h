#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class VoiceInteractionDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(VoiceInteractionDecider);

 public:
  virtual ~VoiceInteractionDecider() override;

  ErrorCode Execute(TaskInfo &task_info) override;
  void SaveTaskResults(TaskInfo &task_info) override{};
  void Reset() override{};

 private:
  bool IsNeedYield(const InsidePlannerData &inside_data,
                   const OutsidePlannerData &outside_data,
                   const PathData &path_data,
                   const DecisionData &decision_data);

  static constexpr double kSpeedEpsilon = 0.2;
};

REGISTER_SCENARIO_TASK(VoiceInteractionDecider);

}  // namespace planning
}  // namespace neodrive
