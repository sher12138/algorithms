#include "navigation_context.h"
#include "navigation_swap_context.h"

namespace neodrive {
namespace planning {

void NavigationContext::Reset() {
  // do not reset road seq when RoutingRequest::NAVIGATER
  if (request_msg.ptr->request_type() != RoutingRequest::NAVIGATER)
    road_seq.clear();
  lane_seq.clear();
  lane_change_seq.clear();
  route_length = 0.0;
  current_s = 0.0;
  is_lane_change_point_updated = false;
  need_lanechange = false;
  lane_change_point_s = 0.0;
  min_lane_change_s = 0.0;
  last_lane_change_finish_s = 0.0;
  one_road_lane_change_s_map.clear();
  is_on_route = true;

  current_utm_ref = nullptr;
  target_utm_ref = nullptr;
  is_swap_ref_line = false;
  navigator_request = NavigatorLaneChangeRequest::NO_REQUEST;
  preview_navigator_request = NavigatorLaneChangeRequest::NO_REQUEST;
}

void NavigationContext::ResetLaneChangeContext() {
  lane_change_seq.clear();
  is_lane_change_point_updated = false;
  need_lanechange = false;
  lane_change_point_s = 0.0;
  min_lane_change_s = 0.0;
  last_lane_change_finish_s = 0.0;
  one_road_lane_change_s_map.clear();
}

}  // namespace planning
}  // namespace neodrive
