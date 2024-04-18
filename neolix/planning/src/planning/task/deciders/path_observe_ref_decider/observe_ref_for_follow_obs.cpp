#include "observe_ref_for_follow_obs.h"

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/config/auto_planning_research_config.h"

namespace neodrive {
namespace planning {

namespace {

enum class Direction { NONE = 0, LEFT, RIGHT, KEEP };

bool ScenarioTrigger(TaskInfo& task_info) {
  return !(task_info.current_frame()
               ->outside_planner_data()
               .path_observe_ref_l_info.left_attention.obstacles.empty() &&
           task_info.current_frame()
               ->outside_planner_data()
               .path_observe_ref_l_info.right_attention.obstacles.empty());
}

std::vector<Direction> SafeDirection(
    TaskInfo& task_info,
    const config::AutoPlanningResearchConfig::PathObserveRefDeciderConfig::
        FollowObs& follow_config) {
  const auto& adc_boundary = task_info.adc_boundary_origin();
  LOG_INFO("adc_boundary: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           adc_boundary.start_s(), adc_boundary.end_s(), adc_boundary.start_l(),
           adc_boundary.end_l());
  const auto& theta = DataCenter::Instance()->vehicle_state_proxy().Heading();
  const auto& observe_info =
      task_info.current_frame()->outside_planner_data().path_observe_ref_l_info;

  /// safe_direction: <0, 1> ==> <left, right>
  std::vector<Direction> safe_direction{Direction::LEFT, Direction::RIGHT};
  for (const auto& obs : observe_info.left_attention.obstacles) {
    const auto& boundary = obs.PolygonBoundary();
    if (boundary.end_s() < adc_boundary.start_s() - follow_config.back_safe_dis)
      continue;
    if (boundary.start_s() >
        adc_boundary.end_s() + follow_config.front_safe_dis)
      continue;
    if (boundary.start_l() - follow_config.lateral_safe_dis >
        adc_boundary.end_l())
      continue;
    if (boundary.end_l() + follow_config.lateral_safe_dis <
        adc_boundary.start_l())
      continue;
    safe_direction[0] = Direction::NONE;
    break;
  }
  for (const auto& obs : observe_info.right_attention.obstacles) {
    const auto& boundary = obs.PolygonBoundary();
    if (boundary.end_s() < adc_boundary.start_s() - follow_config.back_safe_dis)
      continue;
    if (boundary.start_s() >
        adc_boundary.end_s() + follow_config.front_safe_dis)
      continue;
    if (boundary.start_l() - follow_config.lateral_safe_dis >
        adc_boundary.end_l())
      continue;
    if (boundary.end_l() + follow_config.lateral_safe_dis <
        adc_boundary.start_l())
      continue;
    safe_direction[1] = Direction::NONE;
    break;
  }
  safe_direction[1] = (safe_direction[0] == Direction::NONE &&
                       safe_direction[1] == Direction::NONE)
                          ? Direction::KEEP
                          : safe_direction[1];
  LOG_INFO("safe_direction <LEFT, RIGHT>: {}, {}",
           safe_direction[0] == Direction::LEFT,
           safe_direction[1] == Direction::RIGHT);

  return safe_direction;
}

Direction ComputeDirectionWithFollowObstacles(
    TaskInfo& task_info, const std::vector<Direction>& safe_direction,
    const config::AutoPlanningResearchConfig::PathObserveRefDeciderConfig::
        FollowObs& follow_config) {
  /// only consider distance along s-direction
  /// only consider right-direction
  /// TODO:(zhp) sequence obstacles decision may challenge left-direction
  if (safe_direction.back() == Direction::NONE) {
    return Direction::NONE;
  }
  auto& left_obstacles = task_info.current_frame()
                             ->mutable_outside_planner_data()
                             ->path_observe_ref_l_info.left_attention.obstacles;
  std::sort(left_obstacles.begin(), left_obstacles.end(),
            [](const auto& a, const auto& b) {
              return a.PolygonBoundary().start_s() <
                     b.PolygonBoundary().start_s();
            });

  return safe_direction.back();
}

bool ComputeObserveRef(
    TaskInfo& task_info, const Direction& direction,
    const config::AutoPlanningResearchConfig::PathObserveRefDeciderConfig::
        FollowObs& follow_config,
    double* observe_l, std::vector<Obstacle>* attention_dynamic_obstacles,
    PathObserveRefLInfo::RefLType* observe_type) {
  const auto& sl_pt =
      task_info.current_frame()->inside_planner_data().init_sl_point;
  const auto& left_obstacles =
      task_info.current_frame()
          ->mutable_outside_planner_data()
          ->path_observe_ref_l_info.left_attention.obstacles;
  const auto& init_sl_pt =
      task_info.current_frame()->inside_planner_data().init_sl_point;
  if (left_obstacles.empty()) {
    return false;
  }

  double max_l{-1000.0}, min_l{1000.0};
  for (const auto& obs : left_obstacles) {
    max_l = std::max(max_l, obs.PolygonBoundary().end_l());
    min_l = std::min(min_l, obs.PolygonBoundary().start_l());
  }
  const auto& closest_pt = task_info.curr_referline_pt();
  double final_goal_l =
      (direction == Direction::RIGHT || direction == Direction::KEEP)
          ? std::min(0.0, std::max(-closest_pt.right_lane_bound() +
                                       VehicleParam::Instance()->width() * 0.5 +
                                       follow_config.min_road_dis,
                                   min_l -
                                       follow_config.lateral_ratio *
                                           VehicleParam::Instance()->width() -
                                       follow_config.lateral_min_dis))
          : std::max(0.0, std::min(closest_pt.left_lane_bound() -
                                       VehicleParam::Instance()->width() * 0.5 -
                                       follow_config.min_road_dis,
                                   max_l +
                                       follow_config.lateral_ratio *
                                           VehicleParam::Instance()->width() +
                                       follow_config.lateral_min_dis));
  LOG_INFO("max_l, min_l, final_goal_l, init_l: {:.3f}, {:.3f}, {:.3f}, {:.3f}",
           max_l, min_l, final_goal_l, init_sl_pt.l());

  /// smooth observe_l
  *observe_l =
      std::abs(final_goal_l - init_sl_pt.l()) <
              follow_config.max_lateral_change_l
          ? final_goal_l
          : ((final_goal_l > init_sl_pt.l())
                 ? (init_sl_pt.l() + follow_config.max_lateral_change_l)
                 : (init_sl_pt.l() - follow_config.max_lateral_change_l));
  *observe_type = PathObserveRefLInfo::RefLType::FOLLOW;
  LOG_INFO("observe_l: {:.3f}, FOLLOW", *observe_l);

  return true;
}

}  // namespace

ObserveRefForFollowObs::ObserveRefForFollowObs()
    : ObserveRef("ObserveRefForFollowObs") {}

bool ObserveRefForFollowObs::ComputePathObserveRefLInfo(
    TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) {
  if (path_observe_info == nullptr || !observe_ref_config().enable_follow_obs) {
    return false;
  }
  auto& observe_l = path_observe_info->observe_ref_l;
  auto& attention_dynamic_obstacles =
      path_observe_info->attention_dynamic_obstacles;
  auto& observe_type = path_observe_info->type;
  observe_l = 0.;
  attention_dynamic_obstacles.clear();
  observe_type = PathObserveRefLInfo::RefLType::NONE;

  /// scenario trigger
  if (!ScenarioTrigger(task_info)) {
    LOG_INFO("left_attention and right_attention has not follow obstacles");
    return false;
  }

  /// safe direction
  auto safe_direction =
      SafeDirection(task_info, observe_ref_config().follow_obs);

  /// compute direction with reverse obstacles
  auto final_direction = ComputeDirectionWithFollowObstacles(
      task_info, safe_direction, observe_ref_config().follow_obs);
  if (final_direction == Direction::NONE) {
    LOG_INFO("direction with reverse obstacles is NONE");
    return false;
  }

  return ComputeObserveRef(task_info, final_direction,
                           observe_ref_config().follow_obs, &observe_l,
                           &attention_dynamic_obstacles, &observe_type);
}

}  // namespace planning
}  // namespace neodrive
