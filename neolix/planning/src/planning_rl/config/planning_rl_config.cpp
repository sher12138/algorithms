#include "planning_rl_config.h"

#include <sys/stat.h>
#include <unistd.h>

#include "common/file.h"

namespace neodrive {
namespace planning_rl {
namespace config {

PlanningRLConfig::PlanningRLConfig() {
  config_ego_attribuate_state_.file_name =
      "/home/caros/cyberrt/conf/ego_attribuate.json";
  topics_config_state_.file_name =
      "/home/caros/cyberrt/conf/topics_config.json";
  model_config_state_.file_name =
      "/home/caros/cyberrt/conf/rl_model_config.json";
  post_predict_state_.file_name = "/home/caros/cyberrt/conf/post_predict.json";
  ReLoadConfigFromJson();
}

void PlanningRLConfig::ReLoadConfigFromJson() {
  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      config_ego_attribuate_state_.file_name);
  std::cout << "InitAutoEgoAttribuate" << std::endl;
  if (config_ego_attribuate_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        config_ego_attribuate_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoEgoAttribuate(res.second, config_ego_attribuate_);
    // LOG_INFO("InitAutoConfigEgoAttribuate ended. change time: %lu %lu",
    //          config_ego_attribuate_state_.last_change_time, new_change_time);
    config_ego_attribuate_state_.last_change_time = new_change_time;
  }
  std::cout << "InitAutoEgoAttribuate" << std::endl;
  auto new_change_time2 = neodrive::cyber::common::ReadFileChangedTime(
      topics_config_state_.file_name);
  std::cout << "InitAutoTopicsConfig" << std::endl;
  if (topics_config_state_.last_change_time != new_change_time2) {
    auto res =
        neodrive::cyber::common::LoadJsonConfig(topics_config_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoTopicsConfig(res.second, topics_config_);
    // LOG_INFO("AutoTopicsConfig ended. change time: %lu %lu",
    //          topics_config_state_.last_change_time, new_change_time2);
    topics_config_state_.last_change_time = new_change_time2;
  }
  std::cout << "InitAutoTopicsConfig" << std::endl;
  auto new_change_time3 = neodrive::cyber::common::ReadFileChangedTime(
      model_config_state_.file_name);
  if (model_config_state_.last_change_time != new_change_time3) {
    auto res =
        neodrive::cyber::common::LoadJsonConfig(model_config_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoRlModelConfig(res.second, model_config_);
    model_config_state_.last_change_time = new_change_time3;
  }
  std::cout << "InitAutoRlModelConfig" << std::endl;
  auto new_change_time4 = neodrive::cyber::common::ReadFileChangedTime(
      post_predict_state_.file_name);
  if (post_predict_state_.last_change_time != new_change_time4) {
    auto res =
        neodrive::cyber::common::LoadJsonConfig(post_predict_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoPostPredict(res.second, post_predict_);
    post_predict_state_.last_change_time = new_change_time4;
  }
  std::cout << "InitAutoPostPredict" << std::endl;
}

}  // namespace config
}  // namespace planning_rl
}  // namespace neodrive