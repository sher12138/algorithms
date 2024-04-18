#pragma once

#include "common/planning_types.h"

namespace neodrive {
namespace planning {

bool CheckProto(const Chassis& proto);
bool CheckProto(const AebCmd& proto);
bool CheckProto(const RoutingResult& proto);
bool CheckProto(const ControlCommand& proto);
bool CheckProto(const PadNormalCommand& proto);
bool CheckProto(const Freespace& proto);
bool CheckProto(const CameraSegmentation& proto);
bool CheckProto(const DisCode& proto);
bool CheckProto(const PatrolStatus& proto);
bool CheckProto(const PlanningInterface& proto);
bool CheckProto(const LocalizationStatus& proto);
bool CheckProto(const TwistStamped& proto);
bool CheckProto(const PoseStamped& proto);
bool CheckProto(const PerceptionObstacles& proto);
bool CheckProto(const PredictionObstacles& proto);
bool CheckProto(const TrafficLightDetection& proto);
bool CheckProto(const PerceptionLanes& proto);

}  // namespace planning
}  // namespace neodrive
