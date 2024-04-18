#pragma once

#include "common/json_config_util.h"
#include "common_macros.h"
#include "neolix_log.h"
#include "planning/navigation/config/auto_navigation_config.h"
#include "util/time_util.h"

namespace neodrive {
namespace planning {
namespace config {

class NavigationConfig {
  DECLARE_SINGLETON(NavigationConfig);

 public:
  void SetNavigationConfigFilePath(std::string &path);
  ~NavigationConfig() = default;
  void ReLoadConfigFromJson();

  AutoNavigationConfig &navigation_config() { return navigation_config_; }

 private:
  double pre_load_t_ = 0.0;
  static constexpr double kReadJsonInterval_ = 2.0;
  AutoNavigationConfig navigation_config_;
  neodrive::cyber::common::JsonConfigFileStat navigation_config_file_state_;

  friend class NavigationConfigCloud;
};

}  // namespace config
}  // namespace planning
}  // namespace neodrive