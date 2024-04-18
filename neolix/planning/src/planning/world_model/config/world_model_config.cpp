#include "world_model/config/world_model_config.h"

#include <sys/stat.h>
#include <unistd.h>

#include "common/file.h"
#include "cyber.h"

namespace neodrive {
namespace world_model {
namespace config {

WorldModelConfig::WorldModelConfig() {
  world_model_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/world_model_config.json";
  ReLoadConfigFromJson();
}

void WorldModelConfig::ReLoadConfigFromJson() {
  double now_t = cyber::Time::Now().ToSecond();
  if (now_t - pre_load_t_ < kReadJsonInterval_) return;
  pre_load_t_ = now_t;

  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      world_model_config_file_state_.file_name);
  if (world_model_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        world_model_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoWorldModelConfig(res.second, world_model_config_);
    LOG_INFO("InitAutoWorldModelConfig ended. change time: {} {}",
             world_model_config_file_state_.last_change_time, new_change_time);
    world_model_config_file_state_.last_change_time = new_change_time;
  }
}

}  // namespace config
}  // namespace world_model
}  // namespace neodrive