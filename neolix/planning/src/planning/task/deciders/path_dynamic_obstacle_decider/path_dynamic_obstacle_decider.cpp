#include "src/planning/task/deciders/path_dynamic_obstacle_decider/path_dynamic_obstacle_decider.h"

namespace neodrive {
namespace planning {

constexpr double consider_speed{1.5};
constexpr double consider_distance{5};
constexpr double consider_duration{5};
constexpr double consider_resolution{1};
constexpr double consider_rad_diff{M_PI / 6};

namespace {

bool IsObstacleConsider(const Obstacle* obs, TaskInfo& task_info) {
  if (obs->is_virtual() || obs->speed() > consider_speed) return false;
  LOG_INFO("Check obs: {}", obs->id());

  std::size_t consider_size = consider_duration / 0.1 + 0.5;
  auto& traj = obs->prediction_trajectories();
  if (traj.empty() || traj[0].trajectory_points().size() < consider_size)
    return false;

  auto last_point = traj[0].trajectory_points()[consider_size - 1];
  Vec2d lp(last_point.x(), last_point.y());
  SLPoint sl_next{};
  if (!task_info.reference_line()->GetPointInFrenetFrame(lp, &sl_next))
    return false;

  auto sl_curr = obs->center_sl();

  if (sl_curr.s() < task_info.curr_sl().s()) return false;  // behind

  // Difference between lane and trajectory is [+/- 30deg](forward or backward)
  auto rad = std::atan2(sl_next.l() - sl_curr.l(), sl_next.s() - sl_curr.s());
  return std::abs(rad) < consider_rad_diff ||
         std::abs(rad - M_PI) < consider_rad_diff;
}

void SampleObstacleBox(const Obstacle* obs, const ReferenceLinePtr ref_line,
                       std::vector<std::array<double, 4>>* bounds) {
  using AD4 = std::array<double, 4>;
  AD4 t_bound{obs->min_s(), obs->min_l(), obs->max_s(), obs->max_l()};
  bounds->clear();
  for (auto dis = -1e-6; dis < consider_distance; dis += consider_resolution) {
    t_bound[0] += dis;
    t_bound[2] += dis;
    bounds->push_back(t_bound);
  }
}

}  // namespace

PathDynamicObstacleDecider::PathDynamicObstacleDecider() {
  name_ = "PathDynamicObstacleDecider";
}

PathDynamicObstacleDecider::~PathDynamicObstacleDecider() { Reset(); }

ErrorCode PathDynamicObstacleDecider::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  if (!FLAGS_planning_enable_low_speed_obstacles_by_pass) {
    return ErrorCode::PLANNING_SKIP;
  }

  if (task_info.current_frame()->inside_planner_data().is_reverse_driving ||
      task_info.current_frame()->inside_planner_data().is_bias_driving ||
      task_info.current_frame()->inside_planner_data().vel_v > 3.5) {
    LOG_INFO("XXX Don't add atention when reverse or lane borrow or vel > 3.t");
    return ErrorCode::PLANNING_SKIP;
  }
  LOG_INFO("XXX Low speed dynamic obstacle attention");

  auto obstacles = task_info.current_frame()
                       ->planning_data()
                       .decision_data()
                       .dynamic_obstacle();
  auto& attention_boxes = task_info.current_frame()
                              ->mutable_outside_planner_data()
                              ->path_obstacle_context.attention_boxes;
  for (auto& obs : obstacles) {
    if (IsObstacleConsider(obs, task_info)) {
      SampleObstacleBox(obs, task_info.reference_line(), &attention_boxes);
    }
  }
  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
