#pragma once

#include "aeb_msgs.pb.h"
#include "camera_freespace.pb.h"
#include "common/common_macros.h"
#include "control_cmd.pb.h"
#include "cyber.h"
#include "cyberverse.pb.h"
#include "decision.pb.h"
#include "global_adc_status.pb.h"
#include "localization_pose.pb.h"
#include "map_lane.pb.h"
#include "map_road.pb.h"
#include "odometry.pb.h"
#include "pad_msg.pb.h"
#include "patrol_msg.pb.h"
#include "patrol_result.pb.h"
#include "perception_camera.pb.h"
#include "perception_freespace.pb.h"
#include "perception_lane.pb.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
#include "planning_interface.pb.h"
#include "prediction_obstacles.pb.h"
#include "recorder_cmd.pb.h"
#include "routing.pb.h"
#include "scenario_manager_msgs.pb.h"
#include "traffic_light_detection.pb.h"
#include "world_model.pb.h"

namespace neodrive {
namespace planning {

using neodrive::cyber::Component;
using neodrive::cyber::Node;
using neodrive::cyber::Writer;
using neodrive::global::common::PoseStamped;
using neodrive::global::common::Quaternion;
using neodrive::global::common::TwistStamped;
using neodrive::global::control::ControlCommand;
using neodrive::global::control::PadNormalCommand;
using neodrive::global::cyberverse::CyberverseTrigger;
using neodrive::global::data_recorder::EventOfInterest;
using neodrive::global::localization::LocalizationEstimate;
using neodrive::global::patrol::DisCode;
using neodrive::global::patrol::PatrolStatus;
using neodrive::global::perception::CameraSegmentation;
using neodrive::global::perception::Freespace;
using neodrive::global::perception::PerceptionErrorCodeInLidar;
using neodrive::global::perception::SeverityLevel;
using neodrive::global::perception::PerceptionLanes;
using neodrive::global::perception::PerceptionObstacles;
using neodrive::global::perception::SingleCameraSegmentation;
using neodrive::global::perception::camera::CameraLaneLine;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using neodrive::global::planning::ADCSignals;
using neodrive::global::planning::ADCTrajectory;
using neodrive::global::planning::AebCmd;
using neodrive::global::planning::AebState;
using neodrive::global::planning::AutoPilotIntention;
using neodrive::global::planning::DetourStageState;
using neodrive::global::planning::MonitorString;
using neodrive::global::planning::MotorwayDetourStageState;
using neodrive::global::planning::PlanningInterface;
using neodrive::global::planning::ScenarioState;
using neodrive::global::prediction::PredictionObstacles;
using neodrive::global::routing::IsOnMap;
using neodrive::global::routing::RoutingRequest;
using neodrive::global::routing::RoutingResult;
using neodrive::global::status::Chassis;
using neodrive::global::status::DrivingMode;
using neodrive::global::status::GlobalState;
using neodrive::global::world_model::LocalizationStatus;
using SpeedLimitType = global::planning::SpeedLimit;
using SpeedLimitShrPtr = std::shared_ptr<SpeedLimitType>;
using SpeedLimitsType = global::planning::SpeedLimits;
using SpeedLimitShrPtrMap = std::map<double, SpeedLimitShrPtr>;
using neodrive::global::hdmap::BoundaryEdgeType;
using neodrive::global::hdmap::Lane_LaneType;
using neodrive::global::hdmap::LaneBoundaryType_Type;
using neodrive::global::perception::PerceptionObstacle_SubType;
using neodrive::global::planning::MonitorString;
using neodrive::global::status::HornLightsCmd;

}  // namespace planning
}  // namespace neodrive
