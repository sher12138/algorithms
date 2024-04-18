#include "environment.h"

#include "src/planning/common/planning_gflags.h"
#include "src/planning/common/vehicle_param.h"

namespace neodrive {
namespace planning {
using neodrive::global::control::ControlCommand;
using neodrive::global::control::PadNormalCommand;
using neodrive::global::perception::ImpendingCollisionEdges;
using neodrive::global::perception::PerceptionObstacles;
using neodrive::global::perception::freespace::Free_Space_Message;
using neodrive::global::perception::traffic_light::TrafficLightDetection;
using neodrive::global::planning::PlanningInterface;
using neodrive::global::prediction::PredictionObstacles;
using neodrive::global::routing::RoutingResult;
using neodrive::global::status::Chassis;
using neodrive::global::status::GlobalState;
using neodrive::global::status::Status;
using neodrive::global::status::UserCustomMessage;

void Environment::set_vehicle_state(const ChassisShrPtr &chassis) {
  vehicle_state_proxy_.SetVehicleStatus(chassis);
  vehicle_state_odometry_proxy_.SetVehicleStatus(chassis);
}
void Environment::set_vehicle_state(const TwistStampedShrPtr &kinematics) {
  vehicle_state_proxy_.SetVehicleStatus(kinematics);
}
void Environment::set_vehicle_state(const PoseStampedShrPtr &pose) {
  vehicle_state_proxy_.SetVehicleStatus(pose);
}
void Environment::set_vehicle_state(const PoseStampedShrPtr &pose,
                                    const TwistStampedShrPtr &twist) {
  vehicle_state_proxy_.SetVehicleStatus(pose, twist);
}
void Environment::set_vehicle_state_odometry(const PoseStampedShrPtr &pose) {
  vehicle_state_odometry_proxy_.SetVehicleStatus(pose);
}
void Environment::set_vehicle_state_odometry(const PoseStampedShrPtr &pose,
                                             const TwistStampedShrPtr &twist) {
  vehicle_state_odometry_proxy_.SetVehicleStatus(pose, twist);
}

void Environment::set_prediction(const PredictionObstaclesShrPtr &prediction) {
  prediction_proxy_.SetPrediction(prediction);
}

void Environment::set_perception(const PerceptionObstaclesShrPtr &perception) {
  perception_proxy_.SetPerception(perception);
}

void Environment::set_perception_ultrasonic(
    const ImpendingCollisionEdgesShrPtr &perception_ultrasonic) {
  perception_ultrasonic_proxy_.SetPerceptionUltrasonic(perception_ultrasonic);
}

void Environment::set_traffic_light(
    const TrafficLightDetectionShrPtr &traffic_light) {
  traffic_light_proxy_.SetTrafficLight(traffic_light);
}

void Environment::set_perception_lanes(
    const PerceptionLanesShrPtr &perception_lanes) {
  perception_lanes_proxy_.SetPerceptionLanes(perception_lanes);
}

}  // namespace planning
}  // namespace neodrive
