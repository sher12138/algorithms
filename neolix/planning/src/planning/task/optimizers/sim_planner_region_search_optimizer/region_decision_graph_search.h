#pragma once

#include <vector>

#include "common/math/aabox2d.h"
#include "reference_line/reference_line.h"
#include "src/planning/common/data_center/decision_context.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/data_center/path_context.h"
#include "src/planning/common/path/sl_point.h"
#include "src/planning/sim_planner/sim_map.h"
#include "src/planning/sim_planner/state.h"

namespace neodrive {
namespace planning {

class RegionDecisionGraphSearch {
 public:
  std::vector<PathRegion::Bound> GenerateBoundInfo(
      const std::vector<AABox2d>& obstacles,
      const std::vector<SLPoint>& left_lane_bound,
      const std::vector<SLPoint>& right_lane_bound,
      const std::vector<SLPoint>& left_road_bound,
      const std::vector<SLPoint>& right_road_bound, const double init_l,
      const double init_s, ReferenceLinePtr ref_line,
      const std::vector<sim_planner::Vehicle>& sim_traj) const;
};

}  // namespace planning
}  // namespace neodrive
