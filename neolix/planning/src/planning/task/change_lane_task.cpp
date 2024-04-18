#include "change_lane_task.h"


namespace neodrive {
namespace planning {

ChangeLaneTask::ChangeLaneTask() { s_type_name_ = "ChangeLaneTask"; }

const std::string &ChangeLaneTask::Type() const { return s_type_name_; }

ErrorCode ChangeLaneTask::Execute(TaskInfo &task_info,
                                  DecisionData &decision_data,
                                  bool &safe_to_change) {
 return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
