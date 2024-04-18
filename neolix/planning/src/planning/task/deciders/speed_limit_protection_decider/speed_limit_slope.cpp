#include "speed_limit_slope.h"

#include "src/planning/util/speed_limit_trans.h"

namespace neodrive {
namespace planning {

SpeedLimitSlope::SpeedLimitSlope() : SpeedLimitInterface("speed_limit_slope") {}

void SpeedLimitSlope::ComputeSpeedLimit(TaskInfo &task_info) {
  const auto &outside_data = task_info.current_frame()->outside_planner_data();
  const auto &inside_data = task_info.current_frame()->inside_planner_data();
  const auto &reference_line = task_info.reference_line();
  const auto &plan_config = config::PlanningConfig::Instance()->plan_config();
  const double slope_threshold_rad =
      plan_config.common.slope_consider_angle_degree / 180.0 * M_PI;

  double valid_region_s =
      std::max(outside_data.path_context.valid_region_end_s,
               outside_data.path_context.valid_backup_end_s) -
      inside_data.init_sl_point.s();
  auto slope_speed_limit = [](const double slope_rad, const double max_speed,
                              const double slope_threshold_rad) {
    if (std::abs(slope_rad) < slope_threshold_rad) return max_speed;
    if (std::abs(slope_rad) <= 7.0 / 180.0 * M_PI)
      return 4.5;
    else if (std::abs(slope_rad) <= 9.0 / 180.0 * M_PI)
      return 3.0;
    else if (std::abs(slope_rad) <= 12.0 / 180.0 * M_PI)
      return 2.0;
    else
      return 1.5;
  };

  LOG_DEBUG("slope speed limit start");
  double forward_valid_s =
      std::max(1.0, inside_data.init_point.velocity()) * 8.0;
  std::size_t start_index{0}, end_index{0};
  if (!reference_line->GetStartEndIndexBySLength(inside_data.init_sl_point.s(),
                                                 forward_valid_s, &start_index,
                                                 &end_index)) {
    LOG_ERROR("get start/end index failed.");
    return;
  }
  double curr_slope = reference_line->ref_points()[start_index].pitch();

  double curr_slope_limit =
      slope_speed_limit(curr_slope, max_speed(), slope_threshold_rad);
  if (curr_slope_limit >= max_speed() - kMathEpsilon) return;
  double min_end_v{2 * max_speed()}, end_v{2 * max_speed()};
  std::array<double, 3> init_state{0.0, inside_data.init_point.velocity(), 0.0};
  std::array<double, 3> min_end_state{}, closest_end_state{};
  bool closest_find{false};
  for (std::size_t i = start_index; i <= end_index; ++i) {
    const auto &pt = reference_line->ref_points()[i];
    const double forward_s = pt.s() - inside_data.init_sl_point.s();
    if (forward_s > valid_region_s) break;
    if (forward_s > forward_valid_s) break;
    end_v = slope_speed_limit(pt.pitch(), max_speed(), slope_threshold_rad);
    if (end_v < min_end_v && end_v < max_speed()) {
      min_end_state[0] = forward_s;
      min_end_state[1] = end_v;
      min_end_state[2] = 0.;
      min_end_v = end_v;
    }
    if (!closest_find) {
      double closest_end_v =
          slope_speed_limit(pt.pitch(), max_speed(), slope_threshold_rad);
      if (closest_end_v < max_speed()) {
        closest_end_state[0] = forward_s;
        closest_end_state[1] = closest_end_v;
        closest_end_state[2] = 0.;
        closest_find = true;
      }
    }
  }
  if (!closest_find) return;

  /// state of min_end_v
  double min_end_limit = SpeedLimitTrans::InfiniteLimitToSequenceLimit(
      "slope_min_speed_limit", init_state, min_end_state, max_speed(), 3.0);
  min_end_limit = std::max(0., std::min(curr_slope_limit, min_end_limit));

  /// state of closest_end_s
  double closest_end_limit =
      closest_find ? SpeedLimitTrans::InfiniteLimitToSequenceLimit(
                         "slope_closest_speed_limit", init_state,
                         closest_end_state, max_speed(), 3.0)
                   : max_speed();
  closest_end_limit =
      std::max(0., std::min(curr_slope_limit, closest_end_limit));

  double slope_limit = std::min(min_end_limit, closest_end_limit);
  LOG_INFO("slope_limit: {:.2f}", slope_limit);

  if (slope_limit < max_speed()) {
    SaveSpeedLimit(SpeedLimitType::SLOPE, SpeedLimitType::SOFT, slope_limit,
                   0.0);
  }
}

}  // namespace planning
}  // namespace neodrive
