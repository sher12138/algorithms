
#pragma once

#include "global_adc_status.pb.h"
#include "localization_dead_reckoning.pb.h"
#include "localization_error_code.pb.h"
#include "localization_pose.pb.h"
#include "odometry.pb.h"
#include "proto/world_model.pb.h"
#include "sensor_imu.pb.h"

namespace neodrive {
namespace world_model {

using DRResult = neodrive::global::localization_dr::DRResult;
using DRResultShrPtr = std::shared_ptr<DRResult>;
using LocalizationEstimate =
    neodrive::global::localization::LocalizationEstimate;
using LocalizationEstimateShrPtr = std::shared_ptr<LocalizationEstimate>;
using LocalizationStatus = neodrive::global::world_model::LocalizationStatus;
using LocalizationErrorCode =
    neodrive::global::localization::LocalizationErrorCode;
using LocalizationErrorCodeShrPtr = std::shared_ptr<LocalizationErrorCode>;

using TwistStamped = neodrive::global::common::TwistStamped;
using TwistStampedShrPtr = std::shared_ptr<TwistStamped>;
using PoseStamped = neodrive::global::common::PoseStamped;
using PoseStampedShrPtr = std::shared_ptr<PoseStamped>;
using Imu = neodrive::global::drivers::gnss::Imu;
using ImuShrPtr = std::shared_ptr<Imu>;
using Chassis = neodrive::global::status::Chassis;
using ChassisShrPtr = std::shared_ptr<Chassis>;

}  // namespace world_model
}  // namespace neodrive
