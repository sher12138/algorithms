#pragma once

#include <sstream>
#include <vector>

#include "discretized_path.h"
#include "frenet_frame_path.h"
#include "src/planning/common/planning_logger.h"
#include "src/planning/common/planning_macros.h"
#include "reference_line/reference_point.h"

namespace neodrive {
namespace planning {
class PathData {
 public:
  PathData() = default;
  ~PathData() = default;

  bool get_path_point_with_path_s(const double s,
                                  PathPoint *const path_point) const;

  void clear();

  bool is_valid_data() const;

  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(DiscretizedPath, path)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(FrenetFramePath, frenet_path)
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(std::vector<ReferencePoint>,
                                       reference_points)
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, valid_length)
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, heading_dif)
 private:
  DiscretizedPath path_;

  FrenetFramePath frenet_path_;

  std::vector<ReferencePoint> reference_points_;

  double valid_length_ = 0.0;
  double heading_dif_ = 0.0;
};

}  // namespace planning
}  // namespace neodrive
