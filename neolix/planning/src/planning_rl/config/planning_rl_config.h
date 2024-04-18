#pragma once
#include "auto_ego_attribuate.h"
#include "auto_post_predict.h"
#include "auto_rl_model_config.h"
#include "auto_topics_config.h"
#include "common/json_config_util.h"
#include "neolix_log.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {
namespace config {

class PlanningRLConfig {
  DECLARE_PLANNING_SINGLETON(PlanningRLConfig);

 public:
  ~PlanningRLConfig() = default;
  void ReLoadConfigFromJson();

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoEgoAttribuate, config_ego_attribuate);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoTopicsConfig, topics_config);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoRlModelConfig, model_config);

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoPostPredict, post_predict);

 private:
  AutoEgoAttribuate config_ego_attribuate_;
  AutoTopicsConfig topics_config_;
  AutoRlModelConfig model_config_;
  AutoPostPredict post_predict_;
  neodrive::cyber::common::JsonConfigFileStat config_ego_attribuate_state_;
  neodrive::cyber::common::JsonConfigFileStat topics_config_state_;
  neodrive::cyber::common::JsonConfigFileStat model_config_state_;
  neodrive::cyber::common::JsonConfigFileStat post_predict_state_;
};

}  // namespace config
}  // namespace planning_rl
}  // namespace neodrive