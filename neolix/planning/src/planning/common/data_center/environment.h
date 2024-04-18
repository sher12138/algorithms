#pragma once

#include "common_config/config/auto_ego_car_config.h"
#include "config.pb.h"
#include "src/planning/common/planning_macros.h"
#include "src/planning/proxy/perception_lanes_proxy.h"
#include "src/planning/proxy/perception_proxy.h"
#include "src/planning/proxy/perception_ultrasonic_proxy.h"
#include "src/planning/proxy/prediction_proxy.h"
#include "src/planning/proxy/proxy_type.h"
#include "src/planning/proxy/traffic_light_proxy.h"
#include "src/planning/proxy/vehicle_state_proxy.h"

namespace neodrive {
namespace planning {

class Environment {
 public:
  Environment() {}
  ~Environment() {}
  void set_vehicle_state(const ChassisShrPtr &chassis);

  void set_vehicle_state(const TwistStampedShrPtr &kinematics);

  void set_vehicle_state(const PoseStampedShrPtr &pose);
  void set_vehicle_state(const PoseStampedShrPtr &pose,
                         const TwistStampedShrPtr &twist);

  void set_vehicle_state_odometry(const PoseStampedShrPtr &pose);
  void set_vehicle_state_odometry(const PoseStampedShrPtr &pose,
                                  const TwistStampedShrPtr &twist);

  void set_prediction(const PredictionObstaclesShrPtr &prediction);

  void set_perception(const PerceptionObstaclesShrPtr &perception);
  void set_perception_ultrasonic(
      const ImpendingCollisionEdgesShrPtr &perception_ultrasonic);

  void set_traffic_light(const TrafficLightDetectionShrPtr &traffic_light);
  void set_perception_lanes(const PerceptionLanesShrPtr &perception_lanes);

  // get paras
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(VehicleStateProxy, vehicle_state_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(VehicleStateProxy,
                                   vehicle_state_odometry_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(PredictionProxy, prediction_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(PerceptionProxy, perception_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(PerceptionUltrasonicProxy,
                                   perception_ultrasonic_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(TrafficLightProxy, traffic_light_proxy)
  DEFINE_COMPLEX_TYPE_GET_FUNCTION(PerceptionLanesProxy, perception_lanes_proxy)

 private:
  VehicleStateProxy vehicle_state_proxy_{};
  VehicleStateProxy vehicle_state_odometry_proxy_{};
  PredictionProxy prediction_proxy_{};
  PerceptionProxy perception_proxy_{};
  PerceptionUltrasonicProxy perception_ultrasonic_proxy_{};

  TrafficLightProxy traffic_light_proxy_{};
  PerceptionLanesProxy perception_lanes_proxy_{};
};

}  // namespace planning
}  // namespace neodrive
