#pragma once

#include <list>
#include <vector>

#include "src/planning/scenario_manager/scenario_task_interface.h"
#include "src/planning/util/print_util.h"

namespace neodrive {
namespace planning {
class ScenarioTaskPipeline {
 public:
  ScenarioTaskPipeline();
  ~ScenarioTaskPipeline();

 public:
  bool Init(const std::string &stage_name,
            const std::vector<std::string> &name_list,
            const std::unordered_map<std::string, std::vector<std::string>>
                &common_tasks_map);
  ErrorCode ExecuteTasks(TaskInfo &task_info);

 private:
  bool initialized_{false};
  std::string stage_name_;
  std::vector<ScenarioTaskInterface *> scenario_task_list_;
  PrintUtil print_util_;
};

using ScenarioTaskPipelineShrPtr = std::shared_ptr<ScenarioTaskPipeline>;

}  // namespace planning
}  // namespace neodrive