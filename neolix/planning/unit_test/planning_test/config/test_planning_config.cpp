#include <vector>

#include "gtest/gtest.h"
#include "planning/config/planning_config.h"
#include "util/time_logger.h"

namespace neodrive {
namespace planning {

TEST(PlanningConfigTest, test_init) {
  auto planning_config = config::PlanningConfig::Instance();
}

TEST(PlanningConfigTest, test_get_plan_config) {
  auto planning_config = config::PlanningConfig::Instance();
  std::string file_name = "/home/caros/cyberrt/conf/plan_config.json";
  auto res = neodrive::cyber::common::LoadJsonConfig(file_name);

  auto delta_t =
      planning_config->plan_config().inlane_uturn.inlane_uturn_common.delta_t;
  EXPECT_EQ(
      delta_t,
      res.second["inlane_uturn"]["inlane_uturn_common"]["delta_t"].asDouble());
  auto start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  uint64_t run_times = 1000000;
  for (uint64_t i = 0; i < run_times; ++i) {
    auto delta_t =
        planning_config->plan_config().inlane_uturn.inlane_uturn_common.delta_t;
  }
  auto end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("config get time: {}", end_time - start_time);
  start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  for (uint64_t i = 0; i < run_times; ++i) {
    auto delta_t =
        res.second["inlane_uturn"]["inlane_uturn_common"]["delta_t"].asDouble();
  }
  end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("json get time: {}", end_time - start_time);
}

TEST(PlanningConfigTest, test_get_fsm_config) {
  auto planning_config = config::PlanningConfig::Instance();
  std::string file_name = "/home/caros/cyberrt/conf/fsm_config.json";
  auto res = neodrive::cyber::common::LoadJsonConfig(file_name);

  auto planning_init =
      planning_config->fsm_config().fsm_states_pool.planning_init;
  EXPECT_EQ(planning_init,
            res.second["fsm_states_pool"]["PLANNING_INIT"].asString());
  auto start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  uint64_t run_times = 1000000;
  for (uint64_t i = 0; i < run_times; ++i) {
    auto planning_init =
        planning_config->fsm_config().fsm_states_pool.planning_init;
  }
  auto end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("config get time: {}", end_time - start_time);
  start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  for (uint64_t i = 0; i < run_times; ++i) {
    auto planning_init =
        res.second["fsm_states_pool"]["PLANNING_INIT"].asString();
  }
  end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("json get time: {}", end_time - start_time);
}

TEST(PlanningConfigTest, test_get_scene_special_config) {
  auto planning_config = config::PlanningConfig::Instance();
  std::string file_name = "/home/caros/cyberrt/conf/scene_special_config.json";
  auto res = neodrive::cyber::common::LoadJsonConfig(file_name);

  auto x =
      planning_config->scene_special_config().freespace_refer_line_out[0].x;
  EXPECT_EQ(x, res.second["freespace_refer_line_out"][0]["x"].asDouble());
  auto start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  uint64_t run_times = 1000000;
  for (uint64_t i = 0; i < run_times; ++i) {
    auto total_path_length =
        planning_config->scene_special_config().freespace_refer_line_out[0].x;
  }
  auto end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("config get time: {}", end_time - start_time);
  start_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  for (uint64_t i = 0; i < run_times; ++i) {
    auto total_path_length =
        res.second["freespace_refer_line_out"][0]["x"].asDouble();
  }
  end_time = common::util::TimeLogger::GetCurrentTimeMillisecond();
  LOG_INFO("json get time: {}", end_time - start_time);
}

}  // namespace planning
}  // namespace neodrive