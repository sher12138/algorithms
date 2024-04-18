#include "perception_ultrasonic_proxy.h"

namespace neodrive {
namespace planning {

void PerceptionUltrasonicProxy::SetPerceptionUltrasonic(
    const ImpendingCollisionEdgesShrPtr& perception_ultrasonic) {
  SET_PTR(ultrasonic_, perception_ultrasonic, "SetPerceptionUltrasonic");
}

}  // namespace planning
}  // namespace neodrive
