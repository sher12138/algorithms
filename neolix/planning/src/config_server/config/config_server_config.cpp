#include "config/config_server_config.h"

#include <sys/stat.h>
#include <unistd.h>

#include "common/file.h"

namespace neodrive {
namespace config_server {
namespace config {

ConfigServerConfig::ConfigServerConfig() {
  config_server_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/config_server_config.json";
  ReLoadConfigFromJson();
}

void ConfigServerConfig::ReLoadConfigFromJson() {
  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      config_server_config_file_state_.file_name);
  if (config_server_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        config_server_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoConfigServerConfig(res.second, config_server_config_);
    LOG_INFO("InitAutoConfigServerConfig ended. change time: {} {}",
             config_server_config_file_state_.last_change_time,
             new_change_time);
    config_server_config_file_state_.last_change_time = new_change_time;
  }
}

}  // namespace config
}  // namespace config_server
}  // namespace neodrive