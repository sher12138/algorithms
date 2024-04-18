/**
 * @file ego.h
 * data struct define
 **/

#pragma once
#include <json/json.h>
#include "common_geometry.pb.h"
#include "coordinate/coodrdinate_convertion.h"
#include "global_adc_status.pb.h"
#include "localization_pose.pb.h"
#include "neolix_log.h"
#include "planning_map/planning_map.h"
#include "utils/planning_rl_macros.h"

namespace neodrive {
namespace planning_rl {

using neodrive::global::localization::LocalizationEstimate;
using LocalizationEstimateShrPtr = std::shared_ptr<LocalizationEstimate>;

using neodrive::global::common::PoseStamped;
using PoseStampedShrPtr = std::shared_ptr<PoseStamped>;
using neodrive::global::common::TwistStamped;
using TwistStampedShrPtr = std::shared_ptr<TwistStamped>;

using neodrive::global::status::Chassis;
using ChassisShrPtr = std::shared_ptr<Chassis>;
using neodrive::global::common::Point3D;

class Ego {
 public:
  Ego() = default;
  ~Ego() = default;
  Ego(const LocalizationEstimateShrPtr& localization);
  Ego(const PoseStampedShrPtr& odom_pose, const TwistStampedShrPtr& twist);
  Json::Value to_json() const;

  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(double, measurement_time);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(PlanningRLMap::MapPoint, position);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Point3D, velocity);
  DEFINE_COMPLEX_TYPE_GET_SET_FUNCTION(Point3D, linear_acceleration);
  DEFINE_SIMPLE_TYPE_GET_SET_FUNCTION(bool, validate);

 private:
  double measurement_time_;
  PlanningRLMap::MapPoint position_;
  Point3D velocity_;
  Point3D linear_acceleration_;
  bool validate_ = false;
};
// alias for interface using
using Ego2d = std::vector<Ego>;

}  // namespace planning_rl
}  // namespace neodrive
