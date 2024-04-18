#include "event_report_proxy.h"

#include "common/data_center/data_center.h"

namespace neodrive {
namespace planning {

void EventReportProxy::EventReset(const EventType event_type) {
  if (event_map_.find(event_type) != event_map_.end()) {
    Event& event = event_map_[event_type];
    event.start_time = -1.0;
  } else {
    LOG_ERROR("wrong event type: {}", event_type);
  }
}

void EventReportProxy::SetEvent(const EventType event_type) {
  if (event_map_.find(event_type) == event_map_.end()) {
    LOG_ERROR("wrong event type: {}", event_type);
    return;
  }
  Event& event = event_map_[event_type];
  if (!event.enable) {
    LOG_INFO("event<{}> is not enable.", event.name);
    return;
  }
  if (event.presence) {
    LOG_INFO("event<{}> is present.", event.name);
    return;
  }
  if (event_type != EventType::MEET_BARRIER_GATE &&
      event_type == EventType::SPEED_PLAN_FAIL && virtual_obstacle_) {
    LOG_INFO(
        "<{}> won't report because of virtual obstacle from traffic light.",
        event.name);
    return;
  }
  if (DataCenter::Instance()->vehicle_state_proxy().DrivingMode() !=
      neodrive::global::status::DrivingMode::COMPLETE_AUTO_DRIVE) {
    LOG_INFO("The vehicle has been taken over. ({})", event.name);
    return;
  }
  if (event.start_time < kMathEpsilon) {
    event.start_time = DataCenter::Instance()->init_frame_time();
  }
  LOG_INFO("<{}> event start time: {}", event.name, event.start_time);
  if (DataCenter::Instance()->init_frame_time() - event.start_time +
          kMathEpsilon >
      event.min_time) {
    event.presence = true;
    EventInfo* event_info = event_report_.add_event_infos();
    event_info->set_level(event.level);
    event_info->set_name(event.name);
    LOG_INFO("set event level: {}, event name: {}", event.level, event.name);
  }
}

void EventReportProxy::SetEvent(const EventType event_type,
                                const std::string& prefix) {
  if (event_map_.find(event_type) == event_map_.end()) {
    LOG_ERROR("wrong event type: {}", event_type);
    return;
  }
  Event& event = event_map_[event_type];
  if (!event.enable) {
    LOG_INFO("event<{}> is not enable.", event.name);
    return;
  }
  if (event.presence) {
    LOG_INFO("event<{}> is present.", event.name);
    return;
  }
  if (event_type != EventType::MEET_BARRIER_GATE &&
      event_type == EventType::SPEED_PLAN_FAIL && virtual_obstacle_) {
    LOG_INFO(
        "<{}> won't report because of virtual obstacle from traffic light.",
        event.name);
    return;
  }
  if (event.start_time < kMathEpsilon) {
    event.start_time = DataCenter::Instance()->init_frame_time();
  }
  LOG_INFO("<{}> event start time: {}", prefix + event.name, event.start_time);
  if (DataCenter::Instance()->init_frame_time() - event.start_time +
          kMathEpsilon >
      event.min_time) {
    event.presence = true;
    EventInfo* event_info = event_report_.add_event_infos();
    event_info->set_level(event.level);
    event_info->set_name(prefix + event.name);
    LOG_INFO("set event level: {}, event name: {}", event_info->level(),
             event_info->name());
  }
}

void EventReportProxy::ExistVirtualObstacle(
    const std::string& virtual_object_type) {
  virtual_obstacle_ = true;
  LOG_INFO("<{}> created a virtual_obstacle", virtual_object_type);
}

void EventReportProxy::ClearEventInfos() {
  for (auto& event : event_map_) {
    event.second.presence = false;
  }
  virtual_obstacle_ = false;
  event_report_.clear_event_infos();
}

}  // namespace planning
}  // namespace neodrive
