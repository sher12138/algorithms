#pragma once

#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "speed_limit.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/path/path_data.h"
#include "src/planning/common/planning_code_define.h"
#include "src/planning/reference_line/reference_line.h"
#include "st_boundary_config.h"
#include "st_graph_boundary.h"
#include "st_graph_point.h"

namespace neodrive {
namespace planning {

class StBoundaryMapper {
 public:
  StBoundaryMapper(const STBoundaryConfiguration& st_boundary_config);

  const STBoundaryConfiguration& StBoundaryConfig() const;

  ErrorCode GetSpeedLimits(const InsidePlannerData& inside_data,
                           const PathData& path_data,
                           const double& planning_distance,
                           const std::size_t& matrix_dimension_s,
                           const double& default_speed_limit,
                           SpeedLimit* const speed_limit_data) const;

 protected:
  bool CreateYieldBufferBoundaryForReverse(
      const STGraphBoundary& origin_boundary, const double& minimal_time_scope,
      const double& planning_time,
      STGraphBoundary* const buffer_boundary) const;

  bool CreateYieldBufferBoundaryForNonReverse(
      const STGraphBoundary& origin_boundary, const double& minimal_time_scope,
      const double& planning_time,
      STGraphBoundary* const buffer_boundary) const;

 private:
  STBoundaryConfiguration st_boundary_config_;
};

}  // namespace planning
}  // namespace neodrive
