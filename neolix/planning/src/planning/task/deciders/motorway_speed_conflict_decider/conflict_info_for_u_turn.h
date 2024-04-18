#pragma once

#include "conflict_data_base_path.h"
#include "conflict_info_interface.h"

namespace neodrive {
namespace planning {

class ConflictInfoForUTurn final : public ConflictInfoInterface {
 public:
  ConflictInfoForUTurn();
  virtual ~ConflictInfoForUTurn() = default;

  std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) override;

 private:
  std::unordered_set<int> meeting_agent_ids_{};
};

}  // namespace planning
}  // namespace neodrive
