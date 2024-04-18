#include "traffic_light_proxy.h"

namespace neodrive {
namespace planning {

void TrafficLightProxy::SetTrafficLight(
    const TrafficLightDetectionShrPtr& traffic_light) {
  SET_PTR(traffic_light_, traffic_light, "SetTrafficLight");
}

std::vector<TrafficLight> TrafficLightProxy::GetLightsById(
    const uint64_t traffic_light_id) const {
  std::vector<TrafficLight> holder;

  for (auto& light : traffic_light_->traffic_light()) {
    if (light.has_id()) {
      holder.push_back(light);
    }
  }
  return holder;
}

std::vector<neodrive::global::perception::traffic_light::TrafficLight>
TrafficLightProxy::GetLightsNonMotorizedById(
    const uint64_t traffic_light_id) const {
  std::vector<TrafficLight> holder;

  for (auto& light : traffic_light_->traffic_light_non_motorized()) {
    if (light.has_id()) {
      holder.push_back(light);
    }
  }
  return holder;
}

const double TrafficLightProxy::GetTrafficYaw() const {
  if (traffic_light_->has_traffic_yaw())
    return traffic_light_->traffic_yaw();
  else
    return 0.0;
}

const bool TrafficLightProxy::GetUnknownPass() const {
  if (traffic_light_->has_unknow_pass())
    return traffic_light_->unknow_pass();
  else
    return false;
}

}  // namespace planning
}  // namespace neodrive
