#pragma once

#include "config.pb.h"
#include "control_cmd.pb.h"
#include "global_adc_status.pb.h"
#include "localization_dead_reckoning.pb.h"
#include "localization_pose.pb.h"
#include "odometry.pb.h"
#include "pad_msg.pb.h"
#include "perception_camera.pb.h"
#include "perception_free_space.pb.h"
#include "perception_lane.pb.h"
#include "perception_obstacle.pb.h"
#include "perception_ultrasonic.pb.h"
#include "planning.pb.h"
#include "planning_interface.pb.h"
#include "prediction_obstacles.pb.h"
#include "routing.pb.h"
#include "src/planning/common/planning_macros.h"
#include "traffic_light_detection.pb.h"
#include "world_model.pb.h"

namespace neodrive {
namespace planning {

#define SET_PTR(dest, src, str)            \
  {                                        \
    if (src != nullptr) {                  \
      dest = src;                          \
    } else {                               \
      LOG_ERROR("{}: input nullptr", str); \
      return;                              \
    }                                      \
  }

using neodrive::global::routing::RoutingResult;
using RoutingResultShrPtr = std::shared_ptr<RoutingResult>;
using neodrive::global::control::ControlCommand;
using ControlCommandShrPtr = std::shared_ptr<ControlCommand>;
using FreeSpaceMessage =
    neodrive::global::perception::freespace::Free_Space_Message;
using FreeSpaceMessageShrPtr = std::shared_ptr<FreeSpaceMessage>;
using neodrive::global::control::PadNormalCommand;
using PadNormalCommandShrPtr = std::shared_ptr<PadNormalCommand>;
using neodrive::global::status::Status;
using StatusShrPtr = std::shared_ptr<Status>;
using neodrive::global::status::Chassis;
using ChassisShrPtr = std::shared_ptr<Chassis>;
using neodrive::global::planning::PlanningInterface;
using PlanningInterfaceShrPtr = std::shared_ptr<PlanningInterface>;
using neodrive::global::perception::PerceptionObstacle;
using neodrive::global::perception::PerceptionObstacles;
using PerceptionObstaclesShrPtr = std::shared_ptr<PerceptionObstacles>;
using neodrive::global::perception::ImpendingCollisionEdges;
using ImpendingCollisionEdgesShrPtr = std::shared_ptr<ImpendingCollisionEdges>;
using neodrive::global::prediction::PredictionObstacle;
using neodrive::global::prediction::PredictionObstacles;
using PredictionObstaclesShrPtr = std::shared_ptr<PredictionObstacles>;
using neodrive::global::perception::traffic_light::TrafficLight;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using TrafficLightDetectionShrPtr = std::shared_ptr<TrafficLightDetection>;
using neodrive::global::common::TwistStamped;
using TwistStampedShrPtr = std::shared_ptr<TwistStamped>;
using neodrive::global::common::PoseStamped;
using PoseStampedShrPtr = std::shared_ptr<PoseStamped>;
using neodrive::global::world_model::LocalizationStatus;
using LocalizationStatusShrPtr = std::shared_ptr<LocalizationStatus>;
using MsfSecurityLevel = neodrive::global::localization::MsfSecurityLevel;
using neodrive::global::perception::PerceptionLanes;
using PerceptionLanesShrPtr = std::shared_ptr<PerceptionLanes>;
using neodrive::global::perception::camera::CameraLaneLine;
using CameraLaneLinePositionType =
    neodrive::global::perception::camera::LaneLinePositionType;
using CameraLaneLinePoint3d = neodrive::global::perception::camera::Point3D;
using CameraLaneLineType = neodrive::global::perception::camera::LaneLineType;

}  // namespace planning
}  // namespace neodrive
