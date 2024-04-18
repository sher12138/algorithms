#pragma once

#include "aeb/config/auto_aeb_config.h"
#include "aeb/config/auto_dynamic_aeb_config.h"
#include "common/json_config_util.h"
#include "common_macros.h"
#include "neolix_log.h"
#include "util/time_util.h"

namespace neodrive {
namespace aeb {
namespace config {

class AebConfig {
  DECLARE_SINGLETON(AebConfig);

 public:
  ~AebConfig() = default;
  void ReLoadConfigFromJson();

  AutoAebConfig &aeb_config() { return aeb_config_; }
  AutoDynamicAebConfig &aeb_dynamic_config() { return aeb_dynamic_config_; }

 private:
  AutoAebConfig aeb_config_;
  AutoDynamicAebConfig aeb_dynamic_config_;

  double pre_load_t_ = 0.0;
  static constexpr double kReadJsonInterval_ = 2.0;

  neodrive::cyber::common::JsonConfigFileStat aeb_config_file_state_;
  neodrive::cyber::common::JsonConfigFileStat aeb_dynamic_config_file_state_;
};

}  // namespace config
}  // namespace aeb
}  // namespace neodrive