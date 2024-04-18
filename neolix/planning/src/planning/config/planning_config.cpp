#include "config/planning_config.h"

#include "common/file.h"

namespace neodrive {
namespace planning {
namespace config {

PlanningConfig::PlanningConfig() {
  plan_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/plan_config.json";
  fsm_config_file_state_.file_name = "/home/caros/cyberrt/conf/fsm_config.json";
  planning_research_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/planning_research_config.json";
  scene_special_config_file_state_.file_name =
      "/home/caros/cyberrt/conf/scene_special_config.json";
  ReLoadConfigFromJson();
}

void PlanningConfig::ReLoadConfigFromJson() {
  double now_t = common::NowSec();
  if (now_t - pre_load_t_ < kReadJsonInterval_) return;
  pre_load_t_ = now_t;
  auto new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      plan_config_file_state_.file_name);
  if (plan_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        plan_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoPlanConfig(res.second, plan_config_);
    LOG_INFO("InitAutoPlanConfig ended. change time: {} {}",
             plan_config_file_state_.last_change_time, new_change_time);
    plan_config_file_state_.last_change_time = new_change_time;
  }

  new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      fsm_config_file_state_.file_name);
  if (fsm_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        fsm_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoFsmConfig(res.second, fsm_config_);
    LOG_INFO("InitAutoFsmConfig ended. change time: {} {}",
             fsm_config_file_state_.last_change_time, new_change_time);
    fsm_config_file_state_.last_change_time = new_change_time;
  }

  new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      planning_research_config_file_state_.file_name);
  if (planning_research_config_file_state_.last_change_time !=
      new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        planning_research_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoPlanningResearchConfig(res.second, planning_research_config_);
    LOG_INFO("InitAutoPlanningResearchConfig ended. change time: {} {}",
             planning_research_config_file_state_.last_change_time,
             new_change_time);
    planning_research_config_file_state_.last_change_time = new_change_time;
  }

  new_change_time = neodrive::cyber::common::ReadFileChangedTime(
      scene_special_config_file_state_.file_name);
  if (scene_special_config_file_state_.last_change_time != new_change_time) {
    auto res = neodrive::cyber::common::LoadJsonConfig(
        scene_special_config_file_state_.file_name);
    if (!res.first) {
      exit(-1);
    }
    InitAutoSceneSpecialConfig(res.second, scene_special_config_);
    LOG_INFO("InitAutoSceneSpecialConfig ended. change time: {} {}",
             scene_special_config_file_state_.last_change_time,
             new_change_time);
    scene_special_config_file_state_.last_change_time = new_change_time;
  }
}

}  // namespace config
}  // namespace planning
}  // namespace neodrive