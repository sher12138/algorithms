#pragma once

#include "src/planning/common/data_center/outside_planner_data.h"
#include "src/planning/common/math/polygon2d.h"
#include "src/planning/common/math/vec2d.h"
#include "src/planning/common/path/path_point.h"
#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class SpeedCipvOnPathDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(SpeedCipvOnPathDecider);

 public:
  virtual ~SpeedCipvOnPathDecider() override;
  ErrorCode Execute(TaskInfo& task_info) override;
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;
  CipvDecision ReturnResult() const;

 private:
  bool Init(TaskInfo& task_info);
  bool Process(TaskInfo& task_info);
  bool ProcessCipvCheck(TaskInfo& task_info);
  bool CheckDynamicObsInCheckArea(TaskInfo& task_info);
  bool CheckStaticObsInCheckArea(TaskInfo& task_info);
  void GetCipvCheckArea(TaskInfo& task_info);
  void JudgeCipvIsConcerned(TaskInfo& task_info);

 private:
  std::vector<Polygon2d> worm_follow_path_check_area_polygen_;
  double adc_current_s_{0.0};
  double adc_front_edge_s_{0.0};
  DynamicasCipv dynamic_cipv_;
  StaticasCipv static_cipv_;
  // CipvDecision cipv_decision_;
};

REGISTER_SCENARIO_TASK(SpeedCipvOnPathDecider);

}  // namespace planning
}  // namespace neodrive
