#pragma once

#include "src/planning/config/auto_planning_research_config.h"
#include "src/planning/config/planning_config.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class ObserveRef {
 public:
  ObserveRef() = delete;
  virtual ~ObserveRef() = default;

  explicit ObserveRef(const std::string& name) : name_(name) {}

  virtual bool ComputePathObserveRefLInfo(
      TaskInfo& task_info, PathObserveRefLInfo* path_observe_info) = 0;

  const std::string& name() { return name_; }

  const config::AutoPlanningResearchConfig::PathObserveRefDeciderConfig&
  observe_ref_config() {
    return config::PlanningConfig::Instance()
        ->planning_research_config()
        .path_observe_ref_decider_config;
  }

 private:
  std::string name_{""};
};

}  // namespace planning
}  // namespace neodrive
