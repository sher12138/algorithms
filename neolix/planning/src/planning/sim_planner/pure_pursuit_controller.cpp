#include "pure_pursuit_controller.h"

#include <math.h>

namespace neodrive {
namespace planning {
namespace sim_planner {

bool PurePursuitControl::calculateDesiredSteer(const double wheelbase_len,
                                               const double angle_diff,
                                               const double look_ahead_dist,
                                               double *steer) {
  *steer =
      std::atan2(2.0 * wheelbase_len * std::sin(angle_diff), look_ahead_dist);

  return true;
}

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
