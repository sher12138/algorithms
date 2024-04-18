
#pragma once

#include "common/macros.h"
#include "common/message_util.h"
#include "common/util/thread_safe_object.h"
#include "world_model/common/world_model_type.h"

namespace neodrive {
namespace world_model {

enum MsgIndex : int {
  ODOM_POSE_MSG = 0,
  UTM_POSE_MSG,
  IMU_MSG,
  LOC_ERROR_CODE_MSG,
  MAX_ITERM_SIZE
};

class WorldModelContxt {
  DECLARE_SINGLETON(WorldModelContxt);

 public:
  common::util::MessageUnit<Imu> imu_msg;
  common::util::MessageUnit<LocalizationErrorCode> loc_error_code_msg;
  common::util::MessageUnit<DRResult> dr_result_msg;
  common::util::MessageUnit<LocalizationEstimate> localization_estimate_msg;
  common::util::MessageUnit<TwistStamped> kinematics_estimate_msg;
  common::util::MessageUnit<Chassis> chassis_msg;
};

}  // namespace world_model
}  // namespace neodrive
