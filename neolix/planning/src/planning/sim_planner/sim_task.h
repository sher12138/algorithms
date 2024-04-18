#pragma once

namespace neodrive {
namespace planning {
namespace sim_planner {
struct LaneChangeInfo {
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

struct Task {
  bool is_under_ctrl = false;
  double user_desired_vel;
  LaneChangeInfo lc_info;
};
}  // namespace sim_planner
}  // namespace planning
}  // namespace neodrive