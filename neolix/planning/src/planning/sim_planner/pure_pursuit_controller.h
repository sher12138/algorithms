#pragma once
#include "src/planning/common/planning_code_define.h"

namespace neodrive {
namespace planning {
namespace sim_planner {

class PurePursuitControl {
 public:
  static bool calculateDesiredSteer(const double wheelbase_len,
                                    const double angle_diff,
                                    const double look_ahead_dist,
                                    double *steer);
};

}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive
