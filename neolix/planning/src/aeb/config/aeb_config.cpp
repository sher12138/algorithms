#include "config/aeb_config.h"

#include <sys/stat.h>
#include <unistd.h>

#include "common/file.h"

namespace neodrive {
namespace aeb {
namespace config {

AebConfig::AebConfig() {
  aeb_config_file_state_.file_name = "/home/caros/cyberrt/conf/aeb_config.json";
  aeb_dynamic_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/dynamic_aeb_config.json";
  ReLoadConfigFromJson();
}

void AebConfig::ReLoadConfigFromJson() {
  double now_t = common::NowSec();
  if (now_t - pre_load_t_ < kReadJsonInterval_) return;
  pre_load_t_ = now_t;
  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      aeb_config_file_state_.file_name);
  if (aeb_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        aeb_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoAebConfig(res.second, aeb_config_);
    LOG_INFO("InitAutoAebConfig ended. change time: {} {}",
             aeb_config_file_state_.last_change_time, new_change_time);
    aeb_config_file_state_.last_change_time = new_change_time;
  }

  new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      aeb_dynamic_config_file_state_.file_name);
  if (aeb_dynamic_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        aeb_dynamic_config_file_state_.file_name);
    if(!res.first) {
      exit(-1);
    }
    InitAutoDynamicAebConfig(res.second, aeb_dynamic_config_);
    LOG_INFO("InitAutoAebDynamicConfig ended. change time: {} {}",
             aeb_dynamic_config_file_state_.last_change_time, new_change_time);
    aeb_dynamic_config_file_state_.last_change_time = new_change_time;
  }
}

}  // namespace config
}  // namespace aeb
}  // namespace neodrive