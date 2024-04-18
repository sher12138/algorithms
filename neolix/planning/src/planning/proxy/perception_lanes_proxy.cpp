#include "perception_lanes_proxy.h"

#include "src/common/common_macros.h"

namespace neodrive {
namespace planning {

void PerceptionLanesProxy::SetPerceptionLanes(
    const PerceptionLanesShrPtr& perception_lanes) {
  SET_PTR(perception_lanes_, perception_lanes, "SetPerceptionLanes");
  camera_lanes_line_.clear();
  for (const auto& camera_lane_line : perception_lanes->camera_laneline()) {
    if (CheckCameraLaneLine(camera_lane_line)) {
      camera_lanes_line_.push_back(camera_lane_line);
    }
  }
}

bool PerceptionLanesProxy::CheckCameraLaneLine(
    const CameraLaneLine& camera_lane_line) {
  check_proto(camera_lane_line, type);
  check_proto(camera_lane_line, pos_type);
  check_proto(camera_lane_line, curve_camera_coord);
  return true;
}

}  // namespace planning
}  // namespace neodrive