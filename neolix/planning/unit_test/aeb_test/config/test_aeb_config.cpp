#include <vector>

#include "aeb/config/aeb_config.h"
#include "common_config/config/common_config.h"
#include "gtest/gtest.h"
#include "util/time_logger.h"

namespace neodrive {
namespace aeb {
namespace {
TEST(AebConfigTest, test_init) {
  auto aeb_config = config::AebConfig::Instance();
}

TEST(AebConfigTest, test_get_aeb_config) {
  auto aeb_config = config::AebConfig::Instance();
  std::string file_name = "/home/caros/cyberrt/conf/aeb_config.json";
  auto res = neodrive::cyber::common::LoadJsonConfig(file_name);

  EXPECT_EQ(aeb_config->aeb_config().aeb_switch,
            res.second["aeb_switch"].asBool());

  auto start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  uint64_t run_times = 1000000;
  for (uint64_t i = 0; i < run_times; ++i) {
    auto pipeline = aeb_config->aeb_config().aeb_switch;
  }
  auto end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("config get time: {}", end_time - start_time);
  start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  for (uint64_t i = 0; i < run_times; ++i) {
    auto pipeline = res.second["aeb_switch"].asBool();
  }
  end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("json get time: {}", end_time - start_time);
}

TEST(AebConfigTest, test_reload_changed_config_file) {
  int cnt = 0;
  while (cnt < 2) {
    auto aeb_config = config::AebConfig::Instance();
    ++cnt;
    auto start_time = common::util::TimeLogger::GetCurrentTimeseocnd();
    aeb_config->ReLoadConfigFromJson();
    auto end_time = common::util::TimeLogger::GetCurrentTimeseocnd();
    LOG_INFO("ReLoadConfigFromJson time: {}", end_time - start_time);
    LOG_INFO("aeb_period_frequency: {}",
             aeb_config->aeb_config().aeb_period_frequency);
    sleep(1.0);
  }
}

}  // namespace
}  // namespace aeb
}  // namespace neodrive
