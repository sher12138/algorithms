#pragma once

#include <string>
#include <vector>

#include "src/planning/common/data_center/speed_context.h"
#include "src/planning/scenario_manager/scenario_common.h"
#include "src/planning/task/task_info.h"

namespace neodrive {
namespace planning {

class ConflictInfoInterface {
 public:
  explicit ConflictInfoInterface(const std::string& name) : name_(name) {}
  virtual ~ConflictInfoInterface() = default;

  virtual std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) = 0;

  virtual const std::string& name() const { return name_; }

 private:
  std::string name_{""};
};

}  // namespace planning
}  // namespace neodrive
