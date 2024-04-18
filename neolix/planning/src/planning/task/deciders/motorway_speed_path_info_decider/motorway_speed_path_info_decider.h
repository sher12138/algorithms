#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class MotorwaySpeedPathInfoDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(MotorwaySpeedPathInfoDecider);

 public:
  virtual ~MotorwaySpeedPathInfoDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override{};
  void Reset() override{};

 private:
  bool ComputePathAccumulatedS(const PathData& path_data,
                               std::vector<double>* accumulated_s) const;

  bool BuildPathAdcBoundingBoxes(const InsidePlannerData& inside_data,
                                 const PathData& path_data,
                                 std::vector<Box2d>* path_adc_boxes) const;

  bool BuildPathAdcAABoxes(const ReferenceLinePtr ref_ptr,
                           const PathData& path_data,
                           std::vector<Boundary>* adc_aaboxes) const;

  void GetAdcCornerPointCoordinate(
      const InsidePlannerData& inside_data,
      std::vector<Vec2d>& adc_corner_pt_coordinate) const;

  double enlarge_buffer_{0.1};
  double front_enlarge_buffer_{0.1};
  double back_enlarge_buffer_{0.1};
};

REGISTER_SCENARIO_TASK(MotorwaySpeedPathInfoDecider);

}  // namespace planning
}  // namespace neodrive
