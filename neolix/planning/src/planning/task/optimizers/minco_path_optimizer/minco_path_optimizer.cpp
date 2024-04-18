#include "minco_path_optimizer.h"

#include "minco_path_generator.h"

namespace neodrive {
namespace planning {

MincoPathOptimizer::MincoPathOptimizer() { name_ = "MincoPathOptimizer"; }

MincoPathOptimizer::~MincoPathOptimizer() { Reset(); }

ErrorCode MincoPathOptimizer::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  auto& frame = task_info.current_frame();
  if (frame->outside_planner_data().path_succeed_tasks > 0) {
    return ErrorCode::PLANNING_OK;
  }

  MincoPathGenerator path_generator("minco_path_generator");
  if (path_generator.Optimize(
          task_info.reference_line(),
          task_info.current_frame()->inside_planner_data(),
          task_info.current_frame()->mutable_outside_planner_data()) !=
      ErrorCode::PLANNING_OK) {
    LOG_ERROR("minco_path_generator failed.");
    frame->mutable_outside_planner_data()->path_fail_tasks += 1;
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  frame->mutable_outside_planner_data()->path_succeed_tasks += 1;

  return ErrorCode::PLANNING_OK;
}

}  // namespace planning
}  // namespace neodrive
