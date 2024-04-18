#pragma once

#include "common/json_config_util.h"
#include "common/macros.h"
#include "config/auto_config_server_config.h"

namespace neodrive {
namespace config_server {
namespace config {

class ConfigServerConfig {
  DECLARE_SINGLETON(ConfigServerConfig);

 public:
  ~ConfigServerConfig() = default;
  void ReLoadConfigFromJson();

  DEFINE_COMPLEX_TYPE_GET_FUNCTION(AutoConfigServerConfig,
                                   config_server_config);

 private:
  AutoConfigServerConfig config_server_config_;

  neodrive::cyber::common::JsonConfigFileStat config_server_config_file_state_;
};

}  // namespace config
}  // namespace config_server
}  // namespace neodrive