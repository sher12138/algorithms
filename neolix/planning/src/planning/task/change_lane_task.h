#pragma once
/*
 * ChangeLaneTask is used for behavior module, provide path, speed plan's
 * results. Compared to TaskManager, Behavior task only provide rough path/speed
 * results.
 */
#include <vector>

#include "src/planning/common/data_center/data_center.h"
#include "src/planning/common/data_center/inside_planner_data.h"
#include "src/planning/common/data_center/outside_planner_data.h"
#include "task_info.h"

namespace neodrive {
namespace planning {

class ChangeLaneTask final {
 public:
  ChangeLaneTask();
  ~ChangeLaneTask() = default;

  const std::string &Type() const;

  ErrorCode Execute(TaskInfo &task_info, DecisionData &decision_data,
                    bool &safe_to_change);

 private:
  std::string s_type_name_{""};
};

}  // namespace planning
}  // namespace neodrive
