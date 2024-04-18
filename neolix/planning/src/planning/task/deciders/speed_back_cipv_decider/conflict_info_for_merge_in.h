#pragma once

#include "conflict_data_base_map.h"
#include "conflict_data_base_path.h"
#include "conflict_info_interface.h"
#include "src/planning/util/speed_planner_common.h"
namespace neodrive {
namespace planning {

class BackInfoForMergeIn final : public BackInfoInterface {
 public:
  BackInfoForMergeIn();
  virtual ~BackInfoForMergeIn() = default;

  std::vector<ConnectionConflictInfo> ComputeConflictInfo(
      TaskInfo& task_info) override;

 private:
  bool InitialFilter(std::vector<ConnectionConflictInfo>& conflict_info);
  std::unordered_set<int> merge_in_agent_ids_{};
};

}  // namespace planning
}  // namespace neodrive
