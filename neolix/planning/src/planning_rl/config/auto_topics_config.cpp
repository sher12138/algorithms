/**
 * @brief this file is auto-generated by scripts/json_to_cpp.py. Do not edit!!!
 */

#include "config/auto_topics_config.h"

namespace neodrive {
namespace planning_rl {
namespace config {

void InitAutoTopicsConfig(const Json::Value &input_json,
                          AutoTopicsConfig &dest) {
  dest.topics.clear();
  for (const auto &config_dict : input_json["topics"]) {
    AutoTopicsConfig::Topics curr_config{};
    curr_config.key = config_dict["key"].asString();
    curr_config.topic = config_dict["topic"].asString();
    curr_config.queue_size = config_dict["queue_size"].asInt();
    curr_config.type = config_dict["type"].asString();
    dest.topics.insert({curr_config.key, curr_config});
  }
}

}  // namespace config
}  // namespace planning_rl
}  // namespace neodrive
