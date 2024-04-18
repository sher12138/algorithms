#pragma once

#include <Eigen/Geometry>

#include "common/json_config_util.h"
#include "common/macros.h"
#include "world_model/common/world_model_logger.h"
#include "world_model/config/auto_world_model_config.h"

namespace neodrive {
namespace world_model {
namespace config {

class WorldModelConfig {
  DECLARE_SINGLETON(WorldModelConfig);

 public:
  ~WorldModelConfig() = default;
  void ReLoadConfigFromJson();

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoWorldModelConfig, world_model_config);

 private:
  double pre_load_t_ = 0.0;
  static constexpr double kReadJsonInterval_ = 2.0;

  AutoWorldModelConfig world_model_config_;

  neodrive::cyber::common::JsonConfigFileStat world_model_config_file_state_;
};

}  // namespace config
}  // namespace world_model
}  // namespace neodrive