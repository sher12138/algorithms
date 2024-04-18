#pragma once

#include "src/planning/common/st_graph_data/st_graph_boundary.h"

namespace neodrive {
namespace planning {

class AccumulatedSPositionLimit {
 public:
  double ComputeLimit(const std::vector<STGraphBoundary>& st_graph_boundaries,
                      const double init_speed, const double max_speed,
                      const double total_time = 8.0,
                      const double max_deceleration = -6.0,
                      const double max_acceleration = 0.,
                      const double a_resolution = -0.2,
                      const double t_resolution = 0.1,
                      const double preview_time = 4.0);
};

}  // namespace planning
}  // namespace neodrive
