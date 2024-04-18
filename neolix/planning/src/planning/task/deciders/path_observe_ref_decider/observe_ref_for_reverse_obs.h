#pragma once

#include "observe_ref.h"

namespace neodrive {
namespace planning {

class ObserveRefForReverseObs final : public ObserveRef {
 public:
  ObserveRefForReverseObs();

  bool ComputePathObserveRefLInfo(
      TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) override;
};

}  // namespace planning
}  // namespace neodrive
