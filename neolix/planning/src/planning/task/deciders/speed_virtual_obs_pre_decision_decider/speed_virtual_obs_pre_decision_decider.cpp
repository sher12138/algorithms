#include "speed_virtual_obs_pre_decision_decider.h"

#include "src/planning/task/deciders/speed_static_obs_pre_decision_decider/speed_static_obs_pre_decision_decider.h"

namespace neodrive {
namespace planning {

SpeedVirtualObsPreDecisionDecider::SpeedVirtualObsPreDecisionDecider() {
  name_ = "SpeedVirtualObsPreDecisionDecider";
}

SpeedVirtualObsPreDecisionDecider::~SpeedVirtualObsPreDecisionDecider() {
  Reset();
}

ErrorCode SpeedVirtualObsPreDecisionDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks == 0) {
    return ErrorCode::PLANNING_SKIP_REST_TASKS;
  }
  if (!Init(task_info)) {
    LOG_ERROR("Init failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  if (!Process(task_info)) {
    LOG_ERROR("Process failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  return ErrorCode::PLANNING_OK;
}

bool SpeedVirtualObsPreDecisionDecider::Init(TaskInfo& task_info) {
  if (task_info.current_frame()->mutable_outside_planner_data() == nullptr) {
    LOG_ERROR("mutable_outside_planner_data == nullptr.");
    return false;
  }
  if (task_info.current_frame()->outside_planner_data().path_data == nullptr) {
    LOG_ERROR("path_data == nullptr.");
    return false;
  }
  return true;
}

bool SpeedVirtualObsPreDecisionDecider::Process(TaskInfo& task_info) {
  if (SpeedStaticObsPreDecisionDecider::Instance()->VirtualObsPreDecision(
          task_info, task_info.current_frame()->inside_planner_data(),
          task_info.current_frame()
              ->planning_data()
              .decision_data()
              .virtual_obstacle(),
          task_info.current_frame()->mutable_outside_planner_data()) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("Compute virtual obs pre decision decider failed.");
    return false;
  }
  VirtualContextInfo(task_info.current_frame()->mutable_outside_planner_data());

  return true;
}

void SpeedVirtualObsPreDecisionDecider::VirtualContextInfo(
    OutsidePlannerData* const outside_data) {
  LOG_INFO("_speed_obstacle_context_virtual_:");
  for (const auto& context :
       outside_data->speed_obstacle_context.virtual_obstacle_decision) {
    LOG_INFO("obstacle[{}], collided[{}], reverse[{}]", context.obstacle.id(),
             context.collide, context.reverse);
    LOG_INFO("front upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.front().first.s(),
             context.upper_points.front().first.t(),
             context.upper_points.front().second);
    LOG_INFO("back upper_s, upper_t, upper_speed: {:.4f}, {:.4f}, {:.4f}",
             context.upper_points.back().first.s(),
             context.upper_points.back().first.t(),
             context.upper_points.back().second);
    LOG_INFO("front lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.front().first.s(),
             context.lower_points.front().first.t(),
             context.lower_points.front().second);
    LOG_INFO("back lower_s, lower_t, lower_speed: {:.4f}, {:.4f}, {:.4f}",
             context.lower_points.back().first.s(),
             context.lower_points.back().first.t(),
             context.lower_points.back().second);
  }
}

}  // namespace planning
}  // namespace neodrive
