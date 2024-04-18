#pragma once

#include "src/planning/scenario_manager/scenario_task_interface.h"

namespace neodrive {
namespace planning {

class TriggeredStopDecider final : public ScenarioTaskInterface {
  DECLARE_SINGLETON(TriggeredStopDecider);

 public:
  virtual ~TriggeredStopDecider() override;

  ErrorCode Execute(TaskInfo& task_info) override;
  // TODO: need to implemented in future
  void SaveTaskResults(TaskInfo& task_info) override;
  void Reset() override;

 private:
  bool MakeStopDecision(TaskInfo& task_info);

  bool ContinueWithCurrentStopPoint(TaskInfo& task_info);

  bool UpdateTargetStopPoint(TaskInfo& task_info,
                             const ReferencePoint& target_stop_point,
                             ReferencePoint& update_point);

  bool UpdateTargetStopPointByHdmap(TaskInfo& task_info,
                                    const ReferencePoint& target_stop_point,
                                    ReferencePoint& update_point);

 private:
  double current_stop_time_;

  ReferencePoint last_stop_pos_;
  ReferencePoint last_stop_original_pos_;
  global::status::State trigger_cmd_{global::status::INIT};
  global::status::State prev_trigger_cmd_{global::status::INIT};
  // inner state
  bool is_in_pose_adjust_mode_;
  bool is_trigger_stop_{false};
  bool behavior_stop_vehicle_{false};
};

REGISTER_SCENARIO_TASK(TriggeredStopDecider);

}  // namespace planning
}  // namespace neodrive
