#pragma once

#include "src/planning/proxy/proxy_type.h"

namespace neodrive {
namespace planning {

class PerceptionUltrasonicProxy {
 public:
  PerceptionUltrasonicProxy() = default;
  ~PerceptionUltrasonicProxy() = default;

  void SetPerceptionUltrasonic(
      const ImpendingCollisionEdgesShrPtr& perception_ultrasonic);
  const ImpendingCollisionEdges& PerceptionUltrasonic() const {
    return *ultrasonic_;
  }
  double Timestamp() const { return ultrasonic_->header().timestamp_sec(); }

 private:
  ImpendingCollisionEdgesShrPtr ultrasonic_{
      std::make_shared<ImpendingCollisionEdges>()};
};

}  // namespace planning
}  // namespace neodrive
