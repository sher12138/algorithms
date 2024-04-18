#pragma once

#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

class VisualLaneCheckUtmHeading {
 public:
  VisualLaneCheckUtmHeading();

  bool Check();

 private:
  DataCenter* data_center_{nullptr};

  std::vector<Vec2d> left_lane_points_{};
  std::vector<Vec2d> right_lane_points_{};
  std::vector<Vec2d> last_reference_points_{};
};

}  // namespace planning
}  // namespace neodrive