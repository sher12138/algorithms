#include "voice_interaction_decider.h"

#include "src/planning/planning_map/planning_map.h"

namespace neodrive {
namespace planning {

VoiceInteractionDecider::VoiceInteractionDecider() {
  name_ = "VoiceInteractionDecider";
}

VoiceInteractionDecider::~VoiceInteractionDecider() { Reset(); }

ErrorCode VoiceInteractionDecider::Execute(TaskInfo &task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto &frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }

  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &decision_data = frame->planning_data().decision_data();

  data_center_->mutable_global_state_proxy()->SetRequestYield(IsNeedYield(
      inside_data, outside_data, *outside_data.path_data, decision_data));

  return ErrorCode::PLANNING_OK;
}

bool VoiceInteractionDecider::IsNeedYield(
    const InsidePlannerData &inside_data,
    const OutsidePlannerData &outside_data, const PathData &path_data,
    const DecisionData &decision_data) {
  auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  auto &chassis = data_center_->vehicle_state_proxy().chassis();
  if (PlanningMap::Instance()->IsNearJunction(
          inside_data.utm_pose.x(), inside_data.utm_pose.y(),
          plan_config.speed_limit.request_yield_ignore_dist))
    return false;
  if (!data_center_->global_state_proxy().global_state().have_task())
    return false;
  if (chassis.driving_mode() !=
      neodrive::global::status::DrivingMode::COMPLETE_MANUAL)
    return false;
  if (inside_data.vel_v <= kSpeedEpsilon) return false;
  double current_s = inside_data.init_sl_point.s();
  double current_v = inside_data.vel_v;

  double considering_buffer = VehicleParam::Instance()->width();
  double request_length = plan_config.human_interface.request_yield_length;
  if (current_v >= data_center_->drive_strategy_max_speed() * 0.7)
    return false;
  for (const auto &obstacle : decision_data.all_obstacle()) {
    if (obstacle->is_virtual()) continue;
    if (obstacle->is_static() &&
        obstacle->type() != Obstacle::ObstacleType::PEDESTRIAN)
      continue;
    if (obstacle->min_s() > current_s &&
        obstacle->min_s() - current_s < request_length &&
        std::fabs(obstacle->min_l()) < considering_buffer)
      return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive