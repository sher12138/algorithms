#pragma once

#include "world_model/common/world_model_type.h"

namespace neodrive {
namespace world_model {

bool CheckProto(const LocalizationErrorCode& proto);
bool CheckProto(const TwistStamped& proto);
bool CheckProto(const DRResult& proto);
bool CheckProto(const LocalizationEstimate& proto);
bool CheckProto(const Imu& proto);
bool CheckProto(const Chassis& proto);

}  // namespace world_model
}  // namespace neodrive
