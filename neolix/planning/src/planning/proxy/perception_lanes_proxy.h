#pragma once

#include "src/planning/common/math/segment2d.h"
#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class PerceptionLanesProxy {
 public:
  PerceptionLanesProxy() = default;
  ~PerceptionLanesProxy() = default;

  void SetPerceptionLanes(const PerceptionLanesShrPtr& perception_lanes);
  DEFINE_COMPLEX_TYPE_CONST_REF_GET_FUNCTION(std::vector<CameraLaneLine>,
                                             camera_lanes_line)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(std::vector<std::vector<Segment2d>>,
                                   camera_curb_lines)
  const PerceptionLanes &perception_lanes() const { return *perception_lanes_; }

 private:
  bool CheckCameraLaneLine(const CameraLaneLine& lane_line);

  PerceptionLanesShrPtr perception_lanes_{std::make_shared<PerceptionLanes>()};
  std::vector<CameraLaneLine> camera_lanes_line_{};
  std::vector<std::vector<Segment2d>> camera_curb_lines_{};
};
}  // namespace planning
}  // namespace neodrive