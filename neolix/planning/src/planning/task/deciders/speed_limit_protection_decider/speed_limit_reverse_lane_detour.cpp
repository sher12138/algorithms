#include "speed_limit_reverse_lane_detour.h"

#include "common/data_center/data_center.h"
#include "src/planning/util/speed_limit_trans.h"

namespace neodrive {
namespace planning {

SpeedLimitReverseLaneDetour::SpeedLimitReverseLaneDetour()
    : SpeedLimitInterface("speed_limit_reverse_lane_detour") {}

void SpeedLimitReverseLaneDetour::ComputeSpeedLimit(TaskInfo &task_info) {
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &reverse_lane_detour_context =
      DataCenter::Instance()
          ->mutable_master_info()
          ->mutable_reverse_lane_detour_context();
  const auto &conf = config::PlanningConfig::Instance()
                         ->plan_config()
                         .reverse_lane_detour.speed_limit_non_motorway;

  if (!reverse_lane_detour_context->is_allowed_detour_in_reverse_lane) return;

  const double &current_speed = inside_data.init_point.velocity();
  double k = 1.0;
  if (reverse_lane_detour_context->is_left_front_reverse_obs_exist) {
    auto &relative_time_to_reverse_obs =
        reverse_lane_detour_context->relative_time_to_reverse_obs;
    for (auto &obs : relative_time_to_reverse_obs) {
      obs.PrintInfo();
    }
    auto obs = *std::min_element(
        relative_time_to_reverse_obs.begin(),
        relative_time_to_reverse_obs.end(),
        [](auto &s1, auto &s2) { return s1.delt_t < s2.delt_t; });
    double min_t = obs.delt_t;
    if (min_t < 8.0 && min_t > 0.0) {
      k = std::max(0.3, min_t / 8.0);
    }
    LOG_INFO("min_t:{:.4f}, k:{:.4f}", min_t, k);
  }
  double speed_limit;
  switch (reverse_lane_detour_context->adc_position) {
    case ReverseLaneDetourContext::AdcPosition::LEFTMOST_LANE_COMPLETE:
      if (DataCenter::Instance()->master_info().curr_scenario() ==
          ScenarioState::DETOUR) {
        speed_limit = conf.leftmost_lane_complete_limit;
      } else {
        return;
      }
      break;
    case ReverseLaneDetourContext::AdcPosition::DIVIDER_CROSSING:
      speed_limit = conf.divider_crossing_limit;
      break;
    case ReverseLaneDetourContext::AdcPosition::REVERSE_LANE_COMPLETE:
      speed_limit = conf.reverse_lane_complete_limit;
      break;
    case ReverseLaneDetourContext::AdcPosition::OVER_REVERSE_LANE:
      speed_limit = conf.over_reverse_lane_limit;
      LOG_INFO("adc position's over reverse lane!");
      break;
    case ReverseLaneDetourContext::AdcPosition::OVER_LEFTMOST_LANE:
      LOG_INFO("adc position's over leftmost lane!!");
      return;
      break;
    case ReverseLaneDetourContext::AdcPosition::NOT_DEFINE:
      speed_limit = conf.not_define_limit;
      LOG_INFO("adc position's not defined!");
      break;
    default:
      return;
      break;
  }
  LOG_INFO(
      "reverse lane detour speed_limit:{:.4f}, current_speed:{:.4}, "
      "is_obs_near:{}, adc_position:{}",
      speed_limit, current_speed, k != 1.0,
      static_cast<int>(reverse_lane_detour_context->adc_position));
  SaveSpeedLimit(SpeedLimitType::REVERSE_LANE_DETOUR, SpeedLimitType::SOFT,
                 speed_limit, 0.0);
}

}  // namespace planning
}  // namespace neodrive
