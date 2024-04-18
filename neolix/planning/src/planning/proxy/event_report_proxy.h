#pragma once

#include "global_adc_status.pb.h"
#include "planning/config/planning_config.h"

namespace neodrive {
namespace planning {

using EventLevel = neodrive::global::status::EventInfo::EventLevel;
using EventReport = neodrive::global::status::EventReport;
using EventInfo = neodrive::global::status::EventInfo;
enum EventType {
  PREPROCESS_FAIL = 0,
  SPEED_PLAN_FAIL = 1,
  UNKNOWN_TRAFFIC_LIGHT = 2,
  STOP_OVER_LINE = 3,
  PULL_OVER_FAIL = 4,
  MEET_BARRIER_GATE = 5,
  PARK_IN_FINISH = 6,
  PARK_OUT_FINISH = 7,
  PARKING_STUCK = 8,
  PARK_FAIL = 9,
  ABNORMAL_STOP = 10,
  OBSTACLE_BLOCK_TIMEOUT = 11,
  DETOUR_FAIL = 12,
  LANE_CHANGE_FAIL = 13,
  MIXED_LANE_CHANGE_FAIL = 14
};

struct Event {
  EventLevel level;
  std::string name;
  double min_time;
  bool enable;

  double start_time = -1.0;
  bool presence = false;
};

class EventReportProxy {
 public:
  EventReportProxy() {}
  ~EventReportProxy() = default;
  void EventReset(const EventType event_type);
  void SetEvent(const EventType event_type);
  void SetEvent(const EventType event_type, const std::string& prefix);
  void ClearEventInfos();
  void ExistVirtualObstacle(const std::string& virtual_object_type);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(EventReport, event_report);

 private:
  EventReport event_report_;
  config::PlanningConfig* planning_config_ = config::PlanningConfig::Instance();
  bool virtual_obstacle_{false};
  const config::AutoPlanConfig::EventReport& event_conf_ =
      planning_config_->plan_config().event_report;
  std::map<EventType, Event> event_map_ = {
      {EventType::PREPROCESS_FAIL,
       {EventInfo::WARN, "data preprocess failed",
        event_conf_.preprocess_fail_mintime,
        event_conf_.enable_preprocess_fail}},
      {EventType::SPEED_PLAN_FAIL,
       {EventInfo::WARN, "speed plan failed", event_conf_.plan_fail_mintime,
        event_conf_.enable_plan_fail}},
      {EventType::UNKNOWN_TRAFFIC_LIGHT,
       {EventInfo::ERROR, "unknown traffic light",
        event_conf_.unknown_traffic_light_mintime,
        event_conf_.enable_unknown_traffic_light}},
      {EventType::STOP_OVER_LINE,
       {EventInfo::ERROR, "stop over line", event_conf_.stop_over_line_mintime,
        event_conf_.enable_stop_over_line}},
      {EventType::PULL_OVER_FAIL,
       {EventInfo::WARN, "pullover failed", event_conf_.pullover_failed_mintime,
        event_conf_.enable_pullover_failed}},
      {EventType::MEET_BARRIER_GATE,
       {EventInfo::WARN, "meet barrier gate",
        event_conf_.meet_barrier_gate_mintime,
        event_conf_.enable_meet_barrier_gate}},
      {EventType::PARK_IN_FINISH,
       {EventInfo::INFO, ",park_in_finish", event_conf_.parkin_finish_mintime,
        event_conf_.enable_parkin_finish}},
      {EventType::PARK_OUT_FINISH,
       {EventInfo::INFO, ",park_out_finish", event_conf_.parkout_finish_mintime,
        event_conf_.enable_parkout_finish}},
      {EventType::PARK_FAIL,
       {EventInfo::WARN, ",park_failed", event_conf_.park_failed_mintime,
        event_conf_.enable_park_failed}},
      {EventType::ABNORMAL_STOP,
       {EventInfo::WARN, "abnormal_stop", event_conf_.abnormal_stop_mintime,
        event_conf_.enable_abnormal_stop}},
      {EventType::OBSTACLE_BLOCK_TIMEOUT,
       {EventInfo::WARN, "obstacle_block_timeout",
        event_conf_.obstacle_block_timeout_mintime,
        event_conf_.enable_obstacle_block_timeout}},
      {EventType::DETOUR_FAIL,
       {EventInfo::WARN, "detour failed", event_conf_.detour_failed_mintime,
        event_conf_.enable_detour_failed}},
      {EventType::LANE_CHANGE_FAIL,
       {EventInfo::WARN, "lane change failed",
        event_conf_.lane_change_failed_mintime,
        event_conf_.enable_lane_change_failed}},
      {EventType::MIXED_LANE_CHANGE_FAIL,
       {EventInfo::WARN, "mixed lane change failed",
        event_conf_.mixed_lane_change_failed_mintime,
        event_conf_.enable_mixed_lane_change_failed}}};
};

}  // namespace planning
}  // namespace neodrive
