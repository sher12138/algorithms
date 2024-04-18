#pragma once

#include "src/planning/common/planning_code_define.h"
#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class TrafficLightProxy {
 public:
  TrafficLightProxy() = default;
  ~TrafficLightProxy() = default;

  void SetTrafficLight(const TrafficLightDetectionShrPtr& traffic_light);

  std::vector<neodrive::global::perception::traffic_light::TrafficLight>
  GetLightsById(const uint64_t traffic_light_id) const;

  std::vector<neodrive::global::perception::traffic_light::TrafficLight>
  GetLightsNonMotorizedById(const uint64_t traffic_light_id) const;

  const TrafficLightDetection& GetTrafficLight() const {
    return *traffic_light_;
  }
  const double GetTrafficYaw() const;
  const bool GetUnknownPass() const;

  double TimeStamp() const { return traffic_light_->header().timestamp_sec(); }

 private:
  TrafficLightDetectionShrPtr traffic_light_{
      std::make_shared<TrafficLightDetection>()};
};

}  // namespace planning
}  // namespace neodrive
