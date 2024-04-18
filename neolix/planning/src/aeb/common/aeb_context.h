#pragma once
#include "common/aeb_type.h"
#include "common/message_util.h"
#include "env/env.h"

namespace neodrive {
namespace aeb {
class AebContext {
  DECLARE_SINGLETON(AebContext);

 public:
  AebState::State aeb_state{AebState::STANDBY};
  bool is_aeb_active{false};
  bool is_ego_car_stop{false};
  bool is_forbid_aeb{false};
  bool is_failure{false};
  bool is_imu_collision{false};
  bool is_collisioned{false};
  std::vector<Point2D> freespace_pts;
  Environment environment;

 public:
  common::util::MessageUnit<Chassis> chassis_msg;
  common::util::MessageUnit<DRResult> odom_msg;
  common::util::MessageUnit<ForbidAeb> forbid_aeb_msg;
  common::util::MessageUnit<ForbidAeb> pnc_forbid_aeb_msg;
  common::util::MessageUnit<Freespace> freespace_msg;
  common::util::MessageUnit<PatrolStatus> patrol_msg;
  common::util::MessageUnit<PerceptionObstacles> perception_msg;
  common::util::MessageUnit<LocalizationEstimate> localization_msg;
};
}  // namespace aeb
}  // namespace neodrive