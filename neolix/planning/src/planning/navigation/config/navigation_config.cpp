#include "navigation_config.h"

#include <sys/stat.h>
#include <unistd.h>

#include "common/file.h"

namespace neodrive {
namespace planning {
namespace config {

NavigationConfig::NavigationConfig() {
  navigation_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/navigation_config.json";
  ReLoadConfigFromJson();
}

void NavigationConfig::SetNavigationConfigFilePath(std::string &path) {
  navigation_config_file_state_.file_name = path;
  ReLoadConfigFromJson();
}

void NavigationConfig::ReLoadConfigFromJson() {
  double now_t = common::NowSec();
  if (now_t - pre_load_t_ < kReadJsonInterval_) return;
  pre_load_t_ = now_t;
  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      navigation_config_file_state_.file_name);
  if (navigation_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        navigation_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoNavigationConfig(res.second, navigation_config_);
    LOG_INFO("InitAutoNavigationConfig ended. change time: {} {}",
             navigation_config_file_state_.last_change_time, new_change_time);
    navigation_config_file_state_.last_change_time = new_change_time;
  }
}

}  // namespace config
}  // namespace planning
}  // namespace neodrive