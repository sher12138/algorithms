#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class PathLaneBoundaryDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(PathLaneBoundaryDecider);

 public:
  virtual ~PathLaneBoundaryDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override{};

 private:
  bool Init(OutsidePlannerData* const outside_data);

  bool Process(TaskInfo& task_info);

 private:
  bool DynamicDeltaS(const InsidePlannerData& inside_data,
                     OutsidePlannerData* const outside_data) const;

  bool SampleLaneBoundary(const TaskInfo& task_info, const double delta_s,
                          OutsidePlannerData* const outside_data) const;

  void ModifyExtendS(const TaskInfo& task_info, double& end_s) const;

 private:
  static constexpr double kDeltaS = 0.4;
  double prefer_valid_length_;
};

REGISTER_SCENARIO_TASK(PathLaneBoundaryDecider);

}  // namespace planning
}  // namespace neodrive