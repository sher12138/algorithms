#pragma once

#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {
class ScenarioParkCruiseDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioParkCruiseDecider);

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 private:
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioParkCruiseDecider);

}  // namespace planning
}  // namespace neodrive
