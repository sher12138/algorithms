#pragma once

#include "src/planning/common/speed/st_point.h"

namespace neodrive {
namespace planning {

/// TODO: (zhp) should add some soft function to generate st_pts
class STGraphPolyline {
 public:
  STGraphPolyline(const std::vector<STPoint>& st_pts) : st_pts_(st_pts) {}

  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<STPoint>, st_pts);

 private:
  std::vector<STPoint> st_pts_{};
};

}  // namespace planning
}  // namespace neodrive
