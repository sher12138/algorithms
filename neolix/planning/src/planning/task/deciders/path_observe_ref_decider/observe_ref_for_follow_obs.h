#pragma once

#include "observe_ref.h"

namespace neodrive {
namespace planning {

class ObserveRefForFollowObs final : public ObserveRef {
 public:
  ObserveRefForFollowObs();

  bool ComputePathObserveRefLInfo(
      TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) override;
};

}  // namespace planning
}  // namespace neodrive
