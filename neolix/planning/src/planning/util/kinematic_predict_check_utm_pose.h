#pragma once

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/proxy/vehicle_state_proxy.h"

namespace neodrive {
namespace planning {

class KinematicPredictCheckUtmPose {
 public:
  KinematicPredictCheckUtmPose();

  bool Check();

 private:
  DataCenter* data_center_{nullptr};
};

}  // namespace planning
}  // namespace neodrive
