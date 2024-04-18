#include "scenario_task_pipeline.h"

#include "src/planning/common/data_center/data_center.h"

namespace neodrive {
namespace planning {

ScenarioTaskPipeline::ScenarioTaskPipeline() {}

ScenarioTaskPipeline::~ScenarioTaskPipeline() { scenario_task_list_.clear(); }

bool ScenarioTaskPipeline::Init(
    const std::string& stage_name, const std::vector<std::string>& name_list,
    const std::unordered_map<std::string, std::vector<std::string>>&
        common_tasks_map) {
  if (initialized_) {
    return true;
  }
  stage_name_ = stage_name;
  // according to function_task name list, init scenario_task_list_.
  for (auto& task_name : name_list) {
    auto iter = common_tasks_map.find(task_name);
    if (iter != common_tasks_map.end()) {
      auto& task_name_list = iter->second;
      for (auto& sub_task_name : task_name_list) {
        auto task_ptr =
            ScenarioTaskFactory::Instance()->Instantiate(sub_task_name);
        if (task_ptr != nullptr) {
          LOG_INFO("add sub_task for stage: {} {} {}", stage_name_, task_name,
                   sub_task_name);
          scenario_task_list_.emplace_back(task_ptr);
        } else {
          LOG_ERROR("ScenarioTask: {} not found", sub_task_name);
          return false;
        }
      }
    } else {
      auto task_ptr = ScenarioTaskFactory::Instance()->Instantiate(task_name);
      if (task_ptr != nullptr) {
        LOG_INFO("add task for stage: {} {}", stage_name_, task_name);
        scenario_task_list_.emplace_back(task_ptr);
      } else {
        LOG_ERROR("ScenarioTask: {} not found", task_name);
        return false;
      }
    }
  }
  initialized_ = true;
  return true;
}

ErrorCode ScenarioTaskPipeline::ExecuteTasks(TaskInfo& task_info) {
  static DataCenter* data_center = DataCenter::Instance();
  for (auto& task_ptr : scenario_task_list_) {
    auto ret = task_ptr->Execute(task_info);
    task_ptr->SaveTaskResults(task_info);
    print_util_.TimeStatisticTool(task_ptr->Name() + " Done!");
    if (ret == ErrorCode::PLANNING_SKIP_REST_TASKS) {
      LOG_WARN("skip rest tasks: {} {}", stage_name_, task_ptr->Name())
      data_center->mutable_fail_tasks()->push_back(task_ptr->Name());
      return ret;
    } else if (ret != ErrorCode::PLANNING_OK &&
               ret != ErrorCode::PLANNING_SKIP) {
      // only record fail task name and continue to execute the rest tasks
      data_center->mutable_fail_tasks()->push_back(task_ptr->Name());
      LOG_ERROR("ExecuteTasks stage {} task {} error", stage_name_,
                task_ptr->Name());
    }
  }
  print_util_.TimeStatisticPrint(stage_name_);
  return ErrorCode::PLANNING_OK;
}
}  // namespace planning
}  // namespace neodrive
