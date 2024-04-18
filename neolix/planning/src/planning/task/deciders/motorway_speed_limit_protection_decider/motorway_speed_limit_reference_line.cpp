#include "motorway_speed_limit_reference_line.h"

#include "src/planning/util/speed_limit_trans.h"

namespace neodrive {
namespace planning {

MotorwaySpeedLimitReferenceLine::MotorwaySpeedLimitReferenceLine()
    : MotorwaySpeedLimitInterface("motorway_speed_limit_reference_line") {}

void MotorwaySpeedLimitReferenceLine::ComputeSpeedLimit(TaskInfo &task_info) {
  ReferenceLineLimit(task_info);
  JunctionLimit(task_info);
  JunctionDynamicLimit(task_info);
  SpeedBumpLimit(task_info);
}

void MotorwaySpeedLimitReferenceLine::ReferenceLineLimit(TaskInfo &task_info) {
  const auto &reference_line = task_info.reference_line();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  double curr_s_speed_limit = max_speed();
  double current_s = inside_data.init_sl_point.s();
  ReferencePoint ref_pt{};
  if (reference_line->GetNearestRefPoint(current_s, &ref_pt)) {
    curr_s_speed_limit = ref_pt.speed_limit();
  }

  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  double min_end_v{2 * max_speed()}, end_v{2 * max_speed()};
  bool cloest_find{false};
  std::array<double, 3> init_state{0., inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> min_end_state{}, cloest_end_state{};
  for (std::size_t i = 0; i < reference_line->ref_points().size(); i += 10) {
    const auto &pt = reference_line->ref_points().at(i);
    if (pt.s() < current_s) continue;
    double dis = pt.s() - current_s;
    if (dis > forward_valid_s) break;
    end_v = pt.speed_limit();
    if (end_v < min_end_v && end_v < max_speed()) {
      min_end_state[0] = dis;
      min_end_state[1] = end_v;
      min_end_state[2] = 0.0;
      min_end_v = end_v;
    }
    if (!cloest_find) {
      double cloest_end_v = pt.speed_limit();
      if (cloest_end_v < max_speed()) {
        cloest_end_state[0] = dis;
        cloest_end_state[1] = cloest_end_v;
        cloest_end_state[2] = 0.;
        cloest_find = true;
      }
    }
  }
  if (!cloest_find) return;

  /// state of min_end_v
  double min_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "reference_line_min_speed_limit", init_state, min_end_state, max_speed(),
      3.0);
  min_end_limit = std::max(0., std::min(curr_s_speed_limit, min_end_limit));

  /// state of cloest_end_s
  double cloest_end_limit =
      cloest_find ? SpeedLimitTrans::InfiniteLimitToSequenceLimit(
                        "reference_line_cloest_speed_limit", init_state,
                        cloest_end_state, max_speed(), 3.0)
                  : max_speed();
  cloest_end_limit =
      std::max(0., std::min(curr_s_speed_limit, cloest_end_limit));

  double ref_min_limit = std::min(min_end_limit, cloest_end_limit);
  LOG_INFO("ref_min_limit: {:.3f}", ref_min_limit);

  if (ref_min_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::REF_LINE, SpeedLimitType::SOFT,
                   ref_min_limit, 0.0);
  }
}

void MotorwaySpeedLimitReferenceLine::JunctionLimit(TaskInfo &task_info) {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .motor_way.enable_junction_limit) {
    return;
  }
  const auto &reference_line = task_info.reference_line();
  if (reference_line->ref_points().empty()) return;

  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  double current_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  std::array<double, 3> init_state{0., inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> cloest_end_state{};

  bool consider_junction_limit = false;
  for (const auto &overlap : reference_line->junction_overlaps()) {
    if (overlap.end_s < current_s) continue;
    double dis = std::max(overlap.start_s, current_s) - current_s;
    if (dis > forward_valid_s) break;

    consider_junction_limit = true;
    cloest_end_state[0] = dis + 0.1;
    cloest_end_state[1] = common::config::CommonConfig::Instance()
                              ->drive_strategy_config()
                              .non_motorway.junction_limit_speed;
    break;
  }
  if (!consider_junction_limit) return;

  double junction_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "junction_limit", init_state, cloest_end_state, max_speed(), 3.0);
  LOG_INFO("junction_limit: {:.3f}", junction_limit);

  if (junction_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::JUNCTION_SPEEDLIMIT, SpeedLimitType::SOFT,
                   junction_limit, 0.0);
  }
}

void MotorwaySpeedLimitReferenceLine::JunctionDynamicLimit(
    TaskInfo &task_info) {
  if (!common::config::CommonConfig::Instance()
           ->drive_strategy_config()
           .motor_way.enable_dynamic_speed_limit) {
    return;
  }
  if (task_info.curr_referline_pt().lane_type_is_pure_city_driving()) return;
  const auto &reference_line = task_info.reference_line();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  if (reference_line->ref_points().empty()) return;

  double curr_s_speed_limit = max_speed();
  double current_s = inside_data.init_sl_point.s();
  ReferencePoint ref_pt{};
  if (reference_line->GetNearestRefPoint(current_s, &ref_pt)) {
    curr_s_speed_limit = ref_pt.speed_limit();
  }

  /// Filter with range from current_s to ref_points back
  auto junctions_on_reference_line_speed_limits =
      DataCenter::Instance()
          ->mutable_junctions_on_reference_line_speed_limits();
  std::vector<SpeedLimitShrPtr> current_limits;
  auto iter = junctions_on_reference_line_speed_limits->begin();
  while (iter != junctions_on_reference_line_speed_limits->end()) {
    if (current_s > iter->second->zone_end_distance()) {
      junctions_on_reference_line_speed_limits->erase(iter);
      iter = junctions_on_reference_line_speed_limits->begin();
      continue;
    }
    if (iter->first < current_s + reference_line->GetLength()) {
      iter->second->set_upper_bound(common::config::CommonConfig::Instance()
                                        ->drive_strategy_config()
                                        .motor_way.dynamic_limit_speed);
      current_limits.emplace_back(iter->second);
      iter++;
    } else {
      break;
    }
  }
  if (current_limits.empty()) return;

  /// Infinity -> sequence limit
  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  double min_end_v{2 * max_speed()}, end_v{2 * max_speed()};
  bool cloest_find{false};
  std::array<double, 3> init_state{0., inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> min_end_state{}, cloest_end_state{};
  for (std::size_t i = 0; i < current_limits.size(); ++i) {
    const auto &limit = current_limits[i];
    double dis = std::max(limit->zone_start_distance(), current_s) - current_s;
    if (dis > forward_valid_s) break;
    end_v = limit->upper_bound();
    if (end_v < min_end_v && end_v < max_speed()) {
      min_end_state[0] = dis;
      min_end_state[1] = end_v;
      min_end_state[2] = 0.0;
      min_end_v = end_v;
    }
    if (!cloest_find) {
      double cloest_end_v = limit->upper_bound();
      if (cloest_end_v < max_speed()) {
        cloest_end_state[0] = dis;
        cloest_end_state[1] = cloest_end_v;
        cloest_end_state[2] = 0.;
        cloest_find = true;
      }
    }
  }
  if (!cloest_find) return;

  /// state of min_end_v
  double min_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "junction_dynamic_min_speed_limit", init_state, min_end_state,
      max_speed(), 3.0);
  min_end_limit = std::max(0., std::min(curr_s_speed_limit, min_end_limit));
  /// state of cloest_end_s
  double cloest_end_limit =
      cloest_find ? SpeedLimitTrans::InfiniteLimitToSequenceLimit(
                        "junction_dynamic_cloest_speed_limit", init_state,
                        cloest_end_state, max_speed(), 3.0)
                  : max_speed();
  cloest_end_limit =
      std::max(0., std::min(curr_s_speed_limit, cloest_end_limit));

  double junction_limit = std::min(min_end_limit, cloest_end_limit);
  LOG_INFO("junction_dynamic_limit: {:.3f}", junction_limit);

  if (junction_limit < max_speed() && DynamicSpeedLimitCheck(task_info)) {
    SaveSpeedLimit(SpeedLimitType::JUNCTION_DYNAMIC_SPEEDLIMIT,
                   SpeedLimitType::SOFT, junction_limit, 0.0);
  }
}

void MotorwaySpeedLimitReferenceLine::SpeedBumpLimit(TaskInfo &task_info) {
  const auto &reference_line = task_info.reference_line();
  if (reference_line->ref_points().empty()) return;

  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();

  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  double current_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  std::array<double, 3> init_state{0., inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> cloest_end_state{};
  bool consider_speed_bump_limit = false;
  for (const auto &overlap : reference_line->speed_bump_overlaps()) {
    if (overlap.end_s < current_s) continue;
    double dis = std::max(overlap.start_s, current_s) - current_s;
    if (dis > forward_valid_s) break;

    consider_speed_bump_limit = true;
    cloest_end_state[0] = dis;
    cloest_end_state[1] = 2.77;
    break;
  }
  if (!consider_speed_bump_limit) return;

  double speed_bump_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "speed_bump_limit", init_state, cloest_end_state, max_speed(), 3.0);
  LOG_INFO("speed_bump_limit: {:.3f}", speed_bump_limit);

  if (speed_bump_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::SPEED_BUMP, SpeedLimitType::SOFT,
                   speed_bump_limit, 0.0);
  }
}

bool MotorwaySpeedLimitReferenceLine::DynamicSpeedLimitCheck(
    TaskInfo &task_info) {
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  double dynamic_ignore_dist =
      VehicleParam::Instance()->width() / 2.0 +
      plan_config.speed_limit.dynamic_speedlimit_ignore_dist;
  double path_length =
      plan_config.speed_limit.dynamic_speedlimit_consider_length;
  double current_s =
      task_info.current_frame()->inside_planner_data().init_sl_point.s();
  for (const auto &obstacle : task_info.decision_data()->all_obstacle()) {
    if (obstacle->is_virtual()) continue;
    if (obstacle->is_static()) continue;
    if (obstacle->min_s() - current_s > 0.0 &&
        obstacle->min_s() - current_s < path_length &&
        (std::fabs(obstacle->min_l()) < dynamic_ignore_dist ||
         std::fabs(obstacle->max_l()) < dynamic_ignore_dist ||
         (obstacle->min_l() < -dynamic_ignore_dist &&
          obstacle->max_l() > dynamic_ignore_dist)))
      return true;
  }
  return false;
}

}  // namespace planning
}  // namespace neodrive
