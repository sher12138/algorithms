#pragma once
#include "common/json_config_util.h"
#include "planning/config/auto_fsm_config.h"
#include "planning/config/auto_plan_config.h"
#include "planning/config/auto_planning_research_config.h"
#include "planning/config/auto_scene_special_config.h"
#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/planning_macros.h"
#include "util/time_util.h"

namespace neodrive {
namespace planning {
namespace config {

class PlanningConfig {
  DECLARE_SINGLETON(PlanningConfig);

 public:
  ~PlanningConfig() = default;
  void ReLoadConfigFromJson();

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoPlanConfig, plan_config);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoFsmConfig, fsm_config);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoPlanningResearchConfig,
                                   planning_research_config);
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoSceneSpecialConfig,
                                   scene_special_config);

 private:
  AutoPlanConfig plan_config_;
  AutoFsmConfig fsm_config_;
  AutoPlanningResearchConfig planning_research_config_;
  AutoSceneSpecialConfig scene_special_config_;

  double pre_load_t_ = 0.0;
  static constexpr double kReadJsonInterval_ = 2.0;

  neodrive::cyber::common::JsonConfigFileStat plan_config_file_state_;
  neodrive::cyber::common::JsonConfigFileStat fsm_config_file_state_;
  neodrive::cyber::common::JsonConfigFileStat
      planning_research_config_file_state_;
  neodrive::cyber::common::JsonConfigFileStat scene_special_config_file_state_;
};

}  // namespace config
}  // namespace planning
}  // namespace neodrive