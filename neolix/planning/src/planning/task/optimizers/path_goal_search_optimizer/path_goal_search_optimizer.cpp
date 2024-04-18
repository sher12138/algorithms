#include "path_goal_search_optimizer.h"

#include "common/visualizer_event/visualizer_event.h"
#include "path_goal_graph_search.h"

namespace neodrive {
namespace planning {

namespace {

void VisPathGoalPoint(const std::vector<Vec2d>& points) {
  if (!FLAGS_planning_enable_vis_event) return;

  auto event = vis::EventSender::Instance()->GetEvent("path_goal_points");
  event->set_type(visualizer::Event::k3D);
  event->add_attribute(visualizer::Event::kOdom);

  auto set_pts = [](auto event, auto& pts) {
    for (auto& p : pts) {
      auto sphere = event->mutable_sphere()->Add();
      sphere->mutable_center()->set_x(p.x());
      sphere->mutable_center()->set_y(p.y());
      sphere->mutable_center()->set_z(0);
      sphere->set_radius(0.1);
    }
  };

  set_pts(event, points);
}

}  // namespace

PathGoalSearchOptimizer::PathGoalSearchOptimizer() {
  name_ = "PathGoalSearchOptimizer";
}

PathGoalSearchOptimizer::~PathGoalSearchOptimizer() { Reset(); }

ErrorCode PathGoalSearchOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);

  auto decision_data = task_info.decision_data();
  auto reference_line = task_info.reference_line();
  auto outside_data = task_info.current_frame()->mutable_outside_planner_data();
  auto last_path =
      task_info.last_frame()
          ? task_info.last_frame()->outside_planner_data().path_data
          : nullptr;
  const auto& inside_data = task_info.current_frame()->inside_planner_data();

  const auto& lateral_static_obstacles =
      outside_data->path_context.lateral_static_obstacles;
  const auto& bounds_info =
      // outside_data->path_context.original_path_boundary.path_boundary;
      outside_data->path_context.shrink_path_boundary.path_boundary;
  if (bounds_info.empty()) {
    LOG_WARN("bounds_info is empty.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  auto best_goal_sl = PathGoalGraphSearch{}.ComputeBestGoalSL(
      reference_line, inside_data, bounds_info,
      outside_data->path_observe_ref_l_info.observe_ref_l,
      outside_data->path_context.lateral_static_obstacles,
      outside_data->path_observe_ref_l_info.attention_dynamic_obstacles,
      last_path);
  if (best_goal_sl.empty()) {
    LOG_WARN("PathGoalGraphSearch failed.");
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  LOG_INFO("goal path size: {}", best_goal_sl.size());
  outside_data->path_context.path_goal_points.goal_sl_points = best_goal_sl;

  auto& goal_xy_points =
      outside_data->path_context.path_goal_points.goal_xy_points;
  goal_xy_points.clear();
  LOG_INFO("reference line size: {}", reference_line->ref_points().size());
  for (const auto& sl :
       outside_data->path_context.path_goal_points.goal_sl_points) {
    Vec2d xy_point{};
    if (!reference_line->GetPointInCartesianFrame({sl.s(), sl.l()},
                                                  &xy_point)) {
      LOG_ERROR("failed get closest point.");
      continue;
    }
    goal_xy_points.emplace_back(xy_point);
  }
  VisPathGoalPoint(goal_xy_points);

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
