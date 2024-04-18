#pragma once

#include "camera_freespace.pb.h"
#include "common/macros.h"
#include "perception_freespace.pb.h"
#include "src/planning/common/obstacle/obstacle.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {

class FreespaceLaneBoundFitter {
 public:
  FreespaceLaneBoundFitter(
      ReferenceLinePtr ref_line,
      const neodrive::global::perception::Freespace& lidar_freespace,
      const neodrive::global::perception::SingleCameraSegmentation& camera_segments);

 public:
  using AD2 = std::array<double, 2>;
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<AD2>, xy_upper_pts);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<AD2>, xy_lower_pts);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<AD2>, sl_upper_pts);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<AD2>, sl_lower_pts);

 private:
  std::vector<AD2> xy_upper_pts_{};
  std::vector<AD2> xy_lower_pts_{};
  std::vector<AD2> sl_upper_pts_{};
  std::vector<AD2> sl_lower_pts_{};
};

}  // namespace planning
}  // namespace neodrive
