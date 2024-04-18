#pragma once
#include "common/util/thread_safe_object.h"
#include "src/planning/common/data_center/master_info.h"
#include "src/planning/common/parking_space/horizontal_park.h"
#include "src/planning/common/parking_space/oblique_park.h"
#include "src/planning/common/parking_space/vertical_park.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {
enum NavigatorLaneChangeRequest {
  NO_REQUEST = 0,
  REQUEST_LEFT_LANE_CHANGE = 1,
  REQUEST_RIGHT_LANE_CHANGE = 2
};

struct NavigationResult {
  ReferenceLinePtr current_utm_ref{nullptr};
  ReferenceLinePtr target_utm_ref{nullptr};
  bool is_swap_ref_line{false};
  NavigatorLaneChangeRequest navigator_request{
      NavigatorLaneChangeRequest::NO_REQUEST};
  NavigatorLaneChangeRequest preview_navigator_request{
      NavigatorLaneChangeRequest::NO_REQUEST};
  Vec3d lane_change_end_point{};
  double dist_to_lane_change_end{0.0};
};

struct NavigationSwapContext {
  common::util::ThreadSafeObject<ReferencePoint> dest_point;
  common::util::ThreadSafeObject<double> dist_to_end;
  common::util::ThreadSafeObject<double> dist_to_lane_change_end;
  common::util::ThreadSafeObject<double> dist_to_routing_destination;
  common::util::ThreadSafeObject<MasterInfo::DriveDirection> driving_direction;
  common::util::ThreadSafeObject<NavigationResult> navigation_result;
  common::util::ThreadSafeObject<ParkingSpaceShrPtr> parking_space_ptr;
  common::util::ThreadSafeObject<Vec3d> lane_change_end_point;
  common::util::ThreadSafeObject<Vec3d> routing_destion;
};
}  // namespace planning
}  // namespace neodrive