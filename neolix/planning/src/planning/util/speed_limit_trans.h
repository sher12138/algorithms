#pragma once

#include <array>

#include "neolix_log.h"
#include "src/planning/math/curve1d/cubic_polynomial_curve1d.h"

namespace neodrive {
namespace planning {
namespace SpeedLimitTrans {

/// init_state: [s, v, a]
/// end_state:  [s, v, a]
static double InfiniteLimitToSequenceLimit(
    const std::string& source, const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state, const double max_speed,
    double preview_time) {
  LOG_INFO("source: {}", source);
  LOG_INFO("init_state: {:.3f}, {:.3f}, {:.3f}", init_state[0], init_state[1],
           init_state[2]);
  LOG_INFO("end_state: {:.3f}, {:.3f}, {:.3f}", end_state[0], end_state[1],
           end_state[2]);

  if (end_state[1] > max_speed) {
    LOG_WARN("end_state v <= max_speed, could not trans.");
    return max_speed;
  }
  if (end_state[0] - init_state[0] < 1.e-2) {
    LOG_WARN("end_state s <= init_state s, could not trans.");
    return max_speed;
  }

  double virtual_time =
      2 * (end_state[0] - init_state[0]) / (init_state[1] + end_state[1]);
  CubicPolynomialCurve1d cubic_curve1d;
  cubic_curve1d.FitWithEndPointFirstOrder(
      init_state[0], init_state[1], end_state[0], end_state[1], virtual_time);
  double preview_speed =
      cubic_curve1d.Evaluate(1, std::min(preview_time, virtual_time));
  double preview_acc =
      cubic_curve1d.Evaluate(2, std::min(preview_time, virtual_time));

  /// acceleration
  if (preview_acc > 1.e-2) {
    LOG_WARN("preview_acc > 0., should return end_state v");
    return end_state[1];
  }
  /// deceleration
  LOG_INFO(
      "virtual_time, preview_time, preview_speed, preview_acc: {:.3f}, {:.3f}, "
      "{:.3f}, "
      "{:.3f}",
      virtual_time, preview_time, preview_speed, preview_acc);

  return preview_speed;
}

}  // namespace SpeedLimitTrans

}  // namespace planning
}  // namespace neodrive
