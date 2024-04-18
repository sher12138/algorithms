#pragma once

#include "planning/planning_map/planning_map.h"
#include "src/planning/scenario_manager/scenario_common.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class PathOriginalSolveBoundDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathOriginalSolveBoundDecider);

 public:
  virtual ~PathOriginalSolveBoundDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool Init(TaskInfo& task_info);

  bool Process(TaskInfo& task_info);

  bool ExtendLaneBound(TaskInfo& task_info);

  bool ExtendSolveBound(const ReferencePoint& point, const bool left,
                        const bool right, const double ratio,
                        const double* nearby_lane_width, double* left_bound,
                        double* right_bound, bool* left_lane_borrow_flag,
                        bool* right_lane_borrow_flag);

  bool IsAllowedExtendToReverseLane(const ReferencePoint& ref_pt);

  enum class ExtendDirection { NONE = 0, LEFT, RIGHT };
  ExtendDirection extend_direction_{ExtendDirection::NONE};
  bool motorway_detour_{false};
  bool detour_{false};
  double adc_start_l_{0.};
  double adc_end_l_{0.};
};

REGISTER_SCENARIO_TASK(PathOriginalSolveBoundDecider);

}  // namespace planning
}  // namespace neodrive
