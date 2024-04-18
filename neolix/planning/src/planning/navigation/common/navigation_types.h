#pragma once
#include "common/math/vec3d.h"
#include "map_lane.pb.h"
#include "neolix_common/math/aabox2d.h"
#include "neolix_common/math/aaboxkdtree2d.h"
#include "neolix_common/math/box2d.h"
#include "neolix_common/math/line_segment2d.h"
#include "neolix_common/math/math_utils.h"
#include "neolix_common/math/vec2d.h"
#include "odometry.pb.h"
#include "planning_interface.pb.h"
#include "routing.pb.h"
#include "routing_config.pb.h"
#include "routing_topo_graph.pb.h"
#include "src/planning/reference_line/reference_line.h"
namespace neodrive {
namespace planning {
using global::localization::LocalizationEstimate;
using global::planning::PlanningInterface;
using global::routing::RoutingRequest;
using global::routing::RoutingResult;
using global::routing::RoutingResult_ErrorCode;
using global::routing::RoutingResult_LaneChangeInfo;
using global::status::Chassis;
using global::status::DrivingMode;
using global::status::GlobalState;
using global::status::HornLightsCmd;
using neodrive::global::common::PoseStamped;
typedef global::routing::RoutingResult::PassageRegion Passage;
typedef global::routing::RoutingRequest_LaneWaypoint LaneWaypoint;
typedef global::routing::RoutingResult_LaneChangeInfo_Type ChangeLaneType;
}  // namespace planning
}  // namespace neodrive