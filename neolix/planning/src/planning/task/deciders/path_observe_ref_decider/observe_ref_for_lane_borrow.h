#pragma once

#include "observe_ref.h"
#include "src/planning/common/math/aabox2d.h"

namespace neodrive {
namespace planning {

class ObserveRefForLaneBorrow final : public ObserveRef {
 public:
  ObserveRefForLaneBorrow();

  bool ComputePathObserveRefLInfo(
      TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) override;

 private:
  bool ComputeObserveRegions(const InsidePlannerData& inside_data,
                             ReferenceLinePtr ref_ptr);

  void ComputeAttentionDynamicObstacles(
      const std::shared_ptr<DecisionData>& decision_data_ptr,
      const ReferencePoint& ref_pt,
      const LaneBorrowContext::BorrowSide& borrow_side);
  bool ComputeObserveRef(const InsidePlannerData& inside_data,
                         const std::vector<PathRegion::Bound>& bounds_info,
                         const ReferencePoint& ref_pt,
                         const LaneBorrowContext::BorrowSide& borrow_side,
                         double* observe_l,
                         std::vector<Obstacle>* attention_dynamic_obstacles,
                         PathObserveRefLInfo::RefLType* observe_type);

 private:
  using DDDP = std::pair<double, std::pair<double, double>>;
  std::vector<DDDP> borrow_attention_s_vec_{};
  Boundary adc_original_boundary_{};

  std::vector<Obstacle> attention_dynamic_obstacles_{};
  std::vector<AABox2d> attention_lane_bounds_{};
  bool observe_dynamic_obstacles_{false};
};

}  // namespace planning
}  // namespace neodrive
