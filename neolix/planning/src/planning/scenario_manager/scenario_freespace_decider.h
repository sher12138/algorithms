#pragma once

#include "src/planning/scenario_manager/scenario_stage_decider_interface.h"

namespace neodrive {
namespace planning {

class ScenarioFreespaceDecider : public ScenarioStageDeciderInterface {
  DECLARE_SINGLETON(ScenarioFreespaceDecider);

 public:
  bool Init() override;
  bool Reset() override;
  ErrorCode RunOnce() override;

 private:
};

REGISTER_SCENARIO_STAGE_DECIDER(ScenarioFreespaceDecider);

}  // namespace planning
}  // namespace neodrive
