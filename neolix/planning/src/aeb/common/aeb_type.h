#pragma once

#include <memory>

#include "aeb_msgs.pb.h"
#include "car_status.pb.h"
#include "common/common_macros.h"
#include "common_config/config/common_config.h"
#include "config/aeb_config.h"
#include "control_cmd.pb.h"
#include "coordinate/coordinate_math.h"
#include "cyber.h"
#include "decision.pb.h"
#include "global_adc_status.pb.h"
#include "localization_dead_reckoning.pb.h"
#include "patrol_result.pb.h"
#include "perception_freespace.pb.h"
#include "perception_obstacle.pb.h"
#include "planning.pb.h"
#include "prediction_obstacles.pb.h"
#include "recorder_cmd.pb.h"

namespace neodrive {
namespace aeb {
// pb msg type.
using neodrive::global::canbus::PbEpb;
using neodrive::global::common::Point2D;
using neodrive::global::common::Point3D;
using neodrive::global::common::PoseStamped;
using neodrive::global::control::ControlCommand;
using neodrive::global::data_recorder::EventOfInterest;
using neodrive::global::localization::LocalizationEstimate;
using neodrive::global::localization::Pose;
using neodrive::global::patrol::PatrolStatus;
using neodrive::global::perception::Freespace;
using neodrive::global::perception::PerceptionObstacle;
using neodrive::global::perception::PerceptionObstacles;
using neodrive::global::planning::ADCTrajectory;
using neodrive::global::planning::DecisionResult;
using neodrive::global::planning::ForbidAeb;
using neodrive::global::prediction::PredictionObstacles;
using neodrive::global::status::Chassis;
using neodrive::global::status::EventInfo;
using neodrive::global::status::EventReport;
using neodrive::global::status::GEAR_PARKING;
using neodrive::global::status::GEAR_REVERSE;
using neodrive::global::status::GearPosition_Name;
using neodrive::global::status::VCU_ADU;
using neodrive::global::status::VCU_PDU;

using neodrive::common::Distance2d;

using neodrive::aeb::config::AebConfig;
using neodrive::cyber::Node;
using neodrive::cyber::Writer;
using neodrive::global::localization_dr::DRResult;
using neodrive::global::planning::AebChangeFlag;
using neodrive::global::planning::AebCmd;
using neodrive::global::planning::AebState;

typedef std::shared_ptr<PerceptionObstacle> PerceptionObstaclePtr;
typedef std::pair<double, double> SectorType;
}  // namespace aeb
}  // namespace neodrive
