#pragma once
#include "generator.h"
#include "src/planning/proxy/vehicle_state_proxy.h"
#include "src/planning/reference_line/reference_line.h"

namespace neodrive {
namespace planning {
class RefeLineGenerator {
 public:
  typedef std::shared_ptr<planning::ReferenceLine> ReferenceLinePtr;
  RefeLineGenerator(NavigationContext *ctx) : generator_(ctx), ctx_(ctx) {
    utm_ref_ = std::make_shared<ReferenceLine>();
    odometry_ref_ = std::make_shared<ReferenceLine>();
  }

  bool Generate(const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq,
                const TrajectoryPoint &init_pt);
  ReferenceLinePtr TransformToOdometry(const ReferenceLinePtr &utm_ref);
  TrajectoryPoint GetInitPoint(
      const std::vector<cyberverse::LaneInfoConstPtr> &lane_seq);
  TrajectoryPoint GetInitPoint(const VehicleStateProxy &vehicle_state_proxy);
  const Generator GetGenerator() { return generator_; }
  const ReferenceLinePtr GetOdomRef() { return odometry_ref_; }
  const ReferenceLinePtr GetUtmRef() { return utm_ref_; }
  const ReferencePoint &GetDestinationPoint() const {
    return generator_.GetDestinationPoint();
  }
  double GetDistanceToDestination() const {
    return generator_.GetDistanceToDestination();
  }
  double GetDistanceToRefEnd() const {
    return generator_.GetDistanceToRefEnd(utm_ref_);
  }
  double GetCurrentS() const { return generator_.GetCurrentS(); }
  Vec3d GetLaneChangeEndPoint() const {
    return generator_.GetLaneChangeEndPoint();
  }

  void ResetUtmRef() { utm_ref_ = nullptr; }

 private:
  void SwapObject(ReferenceLinePtr &left, ReferenceLinePtr &right);

 private:
  Generator generator_;
  ReferenceLinePtr utm_ref_{};       // utm output ref-line
  ReferenceLinePtr odometry_ref_{};  // odometry output ref-line
  NavigationContext *ctx_{nullptr};
};
}  // namespace planning
}  // namespace neodrive